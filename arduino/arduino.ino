//import libraries
//Sleep Tracking
#include <Time.h>
#include <Wire.h>
//Accelerometer
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//Bluetooth
#include <SoftwareSerial.h>
#include <String.h>

//Accelerometer State Variables
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#define ACCL_LED_PIN 13
MPU6050 mpu;
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Alarm State variables
boolean alarmOn = false;
boolean alarmSet = false;
boolean blueConnected = false;
boolean hasCalibrated = false;

//Sleepmask LED Pins
int leftLED = 4; 
int rightLED = 3;

//Constants
float const pi = 3.1415926;

//Tracking Data
int halfPeriod = 0;
boolean lastIsMin = false;
int sampleSize = 20;
int sampleWait = 1000*2; //2 seconds in milliseconds
//activity amplitudes
float activities[300];
//corresponding times of above activities
int times[300];
//indices of extrema of times. odd is maximum, even is minimum
int extrema[45];
//index of the last that has occurred
int lastActivity = -1;
//index of last extrema that has occurred
int lastExtrema = -1;
//user data
int wakeTime = -1;
int earliestWakeTime;
int latestWakeTime;
//time
int recEarliestTime;
int recLatestTime;

//Alarm Settings
int fadeTime = 10;
int updateWaitPeriod = 1000*60*5; //5 mins in milliseconds
int maxBrightness = 101;//duty cycles

//Bluetooth
#define RxD 11
#define TxD 10

SoftwareSerial BT(RxD, TxD);
String readCache = "";
String message = "";
int midMsg;

void setup(){
    
    /* Uncomment to set time periods suitable for testing */
    //setTestingEnv();

    /* Init Bluetooth */
    BT.begin(9600);
    setupBT();

    /* Init LED */
    //Accelerometer Indicator
    pinMode(ACCL_LED_PIN, OUTPUT);
    //Sleepmask LEDs
    pinMode(leftLED,OUTPUT);
    pinMode(rightLED,OUTPUT);
    
    /*Init Accelerometer*/
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

/* Debug */

void setTestingEnv(){
    fadeTime = 1;
    sampleWait = 100;
    updateWaitPeriod = 5000;
}

void debugState(){
  //current time
  //waketime
  //all extremas
  Serial.println("From Debug State: ");
  Serial.print("  Cur Time is : ");
  Serial.println(now());
  Serial.print("  Wake time is : ");
  Serial.println(wakeTime);
  Serial.print("  Earliest Wake time is : ");
  Serial.println(earliestWakeTime);
  Serial.print("  Latest Wake time is : ");
  Serial.println(latestWakeTime);
  Serial.print("  Half Period is : ");
  Serial.println(halfPeriod);
  
  if(lastExtrema > -1)
    Serial.println("  Extrema:");
  for (int i = 0; i <= lastExtrema; i++){
    Serial.print("    referenced by ");
    Serial.print(i);
    Serial.print(" is at loc ");
    Serial.print(extrema[i]);
    Serial.print(" with value of ");
    Serial.println(activities[extrema[i]]);
  }
}

void debugActivity(){
  //prints lastactivity
  Serial.println("From Debug Activity:");
  Serial.print("  Last Activity is ");
  Serial.println(lastActivity);
  for(int i = 0;i<=lastActivity;i++){
    //print activity
    Serial.print("  at ");
    Serial.print(i);
    Serial.print("; activity is ");
    Serial.print(activities[i]);
    //print time
    Serial.print("; at time ");
    Serial.println(times[i]);
  }
}

void loop(){
    //If alarm is set, we track user data
    if(alarmSet){
        if(!alarmOn){
            debugState();
            
            int curTime = now();
            
            if(curTime >= earliestWakeTime - updateWaitPeriod/1000 && wakeTime == -1)
              findWakeTime();
            if (wakeTime != -1 && curTime - fadeTime*60 >= wakeTime)
              alarmOn = true;
            
            if(!alarmOn){
              //intermitent data collection
              delay(updateWaitPeriod);
              collectData();
              updateExtrema();
              //Reset timing to 0 after first minimum - we define this to be the start of the sleep cycles
              if (lastExtrema == 0 && !hasCalibrated)
                calibrateTrackingData();
            }
        }
        else{
          Serial.println("waking up...");
          fadeLED(fadeTime, 0);
          fadeLED(fadeTime, 1);
          resetAlarm();
        }
    }
    //If alarm not set, we wait to get user input from bluetooth
    else{
        if (BT.available()) {
            while(BT.available()) { //keeps reading the bluetooth
                readCache += (char)BT.read();
            }
            message = readCache;
            readCache = ""; //reset read cache
            if (message.substring(message.indexOf("@"),message.indexOf("@")+1) == "@"){ //checks if @ symbol is in string for safety
                midMsg = message.indexOf("@"); 
                earliestWakeTime = message.substring(1,midMsg).toInt();
                latestWakeTime = message.substring(midMsg+1, message.length()-1).toInt();
                alarmSet = true; //sets alarm
                Serial.print("   Earliest: ");
                Serial.print(earliestWakeTime);
                Serial.print("   Latest: ");
                Serial.println(latestWakeTime);
            }
            else{
                Serial.println("Message received incorrect format " + message);
            }
        }
        delay(100);     
    }
}

//Helper Functions

/*Sleep Tracking*/

void resetAlarm(){
  halfPeriod = 0;
  lastIsMin = false;
  sampleSize = 20;
  lastActivity = -1;
  lastExtrema = -1;
  wakeTime = -1;
  alarmSet = false;
  alarmOn = false;
}

//collect data
void collectData(){
  //Take sampleSize number of samples and return harmonic mean
  float activityScoreDenom = 0;
  int averageDataTime = 0;
  
  for(int i = 0;i<sampleSize;i++){
    // if programming failed, don't try to do anything
    if (!dmpReady) 
      return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
        i--;
        continue;

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        if(i == sampleSize/2)
          averageDataTime = now();
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        
        float accelMagnitude = sqrt((double) (pow(aaWorld.x,2) + pow(aaWorld.y,2) + pow(aaWorld.z,2)));
        activityScoreDenom += 1/accelMagnitude;
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(ACCL_LED_PIN, blinkState);
        
        delay(sampleWait);   
    }
  }
  //Sets times and activities arrays
  lastActivity++;
  times[lastActivity] = averageDataTime; //time 
  float activityScore = sampleSize/activityScoreDenom;
  activities[lastActivity] = activityScore; //activity     
  debugActivity();
}

int getPeriod(){
    return 2*halfPeriod;
}

//Calibrate function to be called when first min is detected
void calibrateTrackingData(){
    Serial.println("calibrating tracking data b/c first min was found...");
    //Phase shift occurs
    int offset = times[lastActivity-3]; 
    setTime(now()-offset);
    
    earliestWakeTime -= offset;
    latestWakeTime -= offset;
    
    //Resetting activity so that first 4 values of activities and corresponding arrays/pointers are 4 values after the first min
    //move activities and times
    for(int i = 0;i<=3;i++){
        activities[i] = activities[lastActivity-3+i];
        times[i] = times[lastActivity-3+i]-offset; //compensating for phase shift
    }
    
    //reset pointers
    lastActivity = 3;
    lastExtrema = 0;
    
    //first min should point to the first activity
    extrema[0] = 0;  
  
    hasCalibrated = true;  
}

//-1 if no extrema, 0 if min, 1 
int updateExtrema(){
    Serial.println("updating extrema...");
    boolean firstHalfIncreasing = true;
    boolean firstHalfDecreasing = true;
    boolean secondHalfIncreasing = true;
    boolean secondHalfDecreasing = true;
    if(lastActivity >= 6){
        for(int i = lastActivity - 6; i < lastActivity-3; i++){
            if(firstHalfIncreasing && activities[i+1] <= activities[i])
              firstHalfIncreasing = false;
            if(firstHalfDecreasing && activities[i+1] > activities[i])
              firstHalfDecreasing = false;
        }
        for(int i = lastActivity - 3; i < lastActivity; i++){
            if(secondHalfIncreasing && activities[i+1] <= activities[i])
              secondHalfIncreasing = false;
            if(secondHalfDecreasing && activities[i+1] > activities[i])
              secondHalfDecreasing = false;
        }
        //found a max
        if (firstHalfIncreasing && secondHalfDecreasing && lastExtrema != -1){
            lastExtrema++;
            extrema[lastExtrema] = lastActivity-3;
            if(lastExtrema > 0)
              updatePeriod();
            lastIsMin = false;
            Serial.print("found a max! ");
            Serial.print(activities[extrema[lastExtrema]]);
            Serial.print(" at time ");
            Serial.println(times[extrema[lastExtrema]]);
            return 1;
        }
        //found a min
        else if (firstHalfDecreasing && secondHalfIncreasing){
            lastExtrema++;  
            extrema[lastExtrema] = lastActivity-3;
            if(lastExtrema > 0)
              updatePeriod();
            lastIsMin = true;
            Serial.print("found a min! ");
            Serial.print(activities[extrema[lastExtrema]]);
            Serial.print(" at time ");
            Serial.println(times[extrema[lastExtrema]]);
            return 0;
        }
        else
          return -1;
    }
}

//update/improve period to reflect current data
void updatePeriod(){
    Serial.println("updating period...");
    if(lastExtrema > 0){
        float total = 0;
        for(int i = 0;i<lastExtrema;i++)
          total += times[extrema[i+1]] - times[extrema[i]];
        halfPeriod = total/lastExtrema;
    }
    Serial.print("new half period is ");
    Serial.println(halfPeriod);
}

//calculates activity at a certain time
//-1 if insufficient data
float predictActivity(float time){
    if (halfPeriod == 0)
    return -1;
    return sin(pi/halfPeriod*time - pi/2);
}

//solves for the next max and returns time
int nextMax(int increment){
    if (lastExtrema % 2 == 1)
      return times[lastExtrema] + 2*halfPeriod*increment;
    else if (lastExtrema % 2 ==0)
      return times[lastExtrema - 1] + 2*halfPeriod*increment;
    else
      return -1;
}

//calculates wake time
void findWakeTime(){
    Serial.println("finding wake time...");
    if(halfPeriod != 0){
      int testMax = nextMax(0);
      int x = 0;
      int predictedLateWake = predictActivity(latestWakeTime);
      int predictedEarlyWake = predictActivity(earliestWakeTime);
      while (testMax < latestWakeTime){
          if (testMax < latestWakeTime && testMax > earliestWakeTime){
              wakeTime = testMax;
              return;
          }else{
              x++;
              testMax = nextMax(x);
          }
      }
      if (testMax > latestWakeTime){
          if (predictedLateWake < predictedEarlyWake)
              wakeTime = predictedEarlyWake;
          else if (predictedLateWake > predictedEarlyWake)
              wakeTime = predictedLateWake;
          else if (predictedLateWake == predictedEarlyWake)
              wakeTime = predictedLateWake;
      }    
    }
    else{
      wakeTime = latestWakeTime;
    }
    
    Serial.print("Wake time set to ");
    Serial.println(wakeTime);
}

/*LED Controller*/

//LED fade; period (between 1- 10 min); state (0 = fade on; 1 = fade off)
void setLEDs(int period, int level){
    analogWrite(leftLED, level);
    analogWrite(rightLED,level);
    delay((period*60000)/(maxBrightness+1));
}

void fadeLED(int period, int state){
    if (state == 0)
      for (int x=0; x<maxBrightness; x++)
        setLEDs(period, x);
    else if (state == 1)
      for (int x = maxBrightness; x>0 ; x--)
        setLEDs(period, x);
}

/* Bluetooth */
//ensures the bluetooth module starts with a name and pass
void setupBT(){
    BT.write("AT+NAMESleepMask");
    delay(1000);
    BT.write("AT+PIN4321");
}

