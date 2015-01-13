Sleep Mask v0.1
====
##What it is:
A sleep mask with embedded LED and networking capabilities. Goal is to provide the best wake up experience personalized by sleep cycle tracking and smart LED controls.

##Features:
* Android app that communicates to the Sleep Mask device via bluetooth.
* Able to set earliest and latest wake up time alarms.
* Sleep Mask device is powered by the Arduino.
* Automatic Sleep Tracking using accelerometer.
* Automatic best wake up time determination loosely based on trignometric regression.

##Needed Hardware:

* Arduino Mega
* MPU-6050 (GY-521 Breakout Board)
* JY-MCU Arduino Bluetooth Module
* 2 LED Pins
* Sleep Mask

##Pin Placements:
* 3 and 4 for LEDs
* 13 for Accelerometer Indicator Light
* 11 for Bluetooth RxD
* 12 for Bluetooth TxD
* 2 for MPU-6050 Interrupt 
* 20 for MPU-6050 SDA
* 21 for MPPU-6050 SCL

##Needed External Arduino Libraries:
* MPU-6050
* I2C
* Time

*Note: due to Arduino Mega's power design, the accelerometer  needs to be powered by 3.3V. Bluetooth module is powered by 5V.*

##Sleep Tracking Algorithm:
1. Collect accelerometer data on a periodic basis with multiple samples every time. Convert raw data into an activity score.
2. Proceed with periodic data collection. When the first local minimum is reached, shift all time and activity recordings accordingly such that the first minimum is the first activity on record. 

   This is to eliminate unreliable activity data recorded before the user falls asleep.

3. Then, every time a new minimum or maximum is found, calculate a new half period with the most recent data. We use harmonic means in our algorithm to reduce the influence of unreliable outliers (not uncommon with accelerometer measurements).
4. At a time before the user-defined earliest wake up time, calculate whether or not a maximum exists using trigonometric regression. 
5. If such maximum exists, then wake user up at that time. If it doesn't exist, wake the user up at either the earliest or the latest wake up time, whichever one is predicted to have a higher activity score.

###Special Thanks to Jeff Rowberg's MPU-6050 library!!