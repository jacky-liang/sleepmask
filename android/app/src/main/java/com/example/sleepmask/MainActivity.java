package com.example.sleepmask;

import android.os.Bundle;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import android.content.Intent;
import android.content.IntentFilter;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;
import android.support.v4.app.DialogFragment;
import android.support.v4.app.FragmentActivity;
import android.text.format.DateFormat;

import com.example.sleepmask.TimePickerFragment.TimePickedListener;

import java.util.Calendar;
import java.util.UUID;

public class MainActivity extends FragmentActivity implements TimePickedListener {

    private static final int REQUEST_ENABLE_BT = 1;
    private Button onBtn;
    private Button offBtn;
    private Button listBtn;
    private Button findBtn;
    private TextView text;
    private TextView status;
    private BluetoothAdapter myBluetoothAdapter;
    private BluetoothSocket btSocket = null;
    private Set<BluetoothDevice> pairedDevices;
    private ListView myListView;
    private ArrayAdapter<String> BTArrayAdapter;
    private ConnectedThread mConnectedThread;
    private boolean connected = false;

    // SPP UUID service - this should work for most devices
    private static final UUID BTMODULEUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    // identifier used to distinguish sleep mask bluetooth devices
    // can be revised
    private static String identifier = "30:14:11:27:02:01";

    //Alarm stuff
    private TextView mEarliestTimeText;
    private Button mEarliestTimeButton;
    private TextView mLatestTimeText;
    private Button mLatestTimeButton;
    private TextView mTargetTimeText;
    private Button mSetAlarmButton;
    private Calendar earliestTime = Calendar.getInstance();
    private Calendar latestTime = Calendar.getInstance();
    private Calendar targetTime;
    private boolean earliestTimeIsSet = false;
    private boolean latestTimeIsSet = false;

    @Override
    public void onTimePicked(Calendar time)
    {
        mTargetTimeText.setText(DateFormat.format("h:mm a", time));
        targetTime.setTimeInMillis(time.getTimeInMillis());
    }

    public boolean isProperDevice(BluetoothDevice device){
        return device.getAddress().equals(identifier);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //Link activity with corresponding layout xml
        setContentView(R.layout.activity_main);

        // take an instance of BluetoothAdapter - Bluetooth radio
        myBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if(myBluetoothAdapter == null) {
            //setenabled enables buttons
            onBtn.setEnabled(false);
            offBtn.setEnabled(false);
            listBtn.setEnabled(false);
            findBtn.setEnabled(false);
            mEarliestTimeButton.setEnabled(false);
            mLatestTimeButton.setEnabled(false);
            mSetAlarmButton.setEnabled(false);

            text.setText("Status: not supported");

            //toast is an on screen notification
            Toast.makeText(getApplicationContext(),"Your device does not support Bluetooth",
                    Toast.LENGTH_LONG).show();
        } else {
            //Blutooth interaction

            status = (TextView) findViewById(R.id.status);
            text = (TextView) findViewById(R.id.text);
            text.setText("Bluetooth Status: "+ Integer.toString(myBluetoothAdapter.getState()));

            onBtn = (Button) findViewById(R.id.turnOn);

            onBtn.setOnClickListener(new OnClickListener() {
                @Override
                public void onClick(View v) {
                    on(v);
                }
            });

            offBtn = (Button)findViewById(R.id.turnOff);
            offBtn.setOnClickListener(new OnClickListener() {
                @Override
                public void onClick(View v) {
                    off(v);
                }
            });

            listBtn = (Button)findViewById(R.id.paired);
            listBtn.setOnClickListener(new OnClickListener() {
                @Override
                public void onClick(View v) {
                    list(v);
                }
            });

            findBtn = (Button)findViewById(R.id.search);
            findBtn.setOnClickListener(new OnClickListener() {
                @Override
                public void onClick(View v) {
                    find(v);
                }
            });

            myListView = (ListView)findViewById(R.id.listView1);

            // create the arrayAdapter that contains the BTDevices, and set it to the ListView
            BTArrayAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1);
            myListView.setAdapter(BTArrayAdapter);

            //Alarm interaction
            mEarliestTimeText = (TextView) findViewById(R.id.text_earliest_time);
            mEarliestTimeButton = (Button) findViewById(R.id.button_earliest_time);
            mLatestTimeText = (TextView) findViewById(R.id.text_latest_time);
            mLatestTimeButton = (Button) findViewById(R.id.button_latest_time);
            mSetAlarmButton = (Button) findViewById(R.id.button_set_alarm);

            mSetAlarmButton.setOnClickListener(new OnClickListener() {
                @Override
                public void onClick(View v) {

                    if(!earliestTimeIsSet || !latestTimeIsSet){
                        status.setText("Please set both times");
                        return;
                    }

                    long now = System.currentTimeMillis();

                    long now_to_earliest = (earliestTime.getTimeInMillis() - now)/1000;//seconds
                    if(now_to_earliest < 0)
                        now_to_earliest += 8600;

                    long now_to_latest = (latestTime.getTimeInMillis() - now)/1000;
                    if(now_to_latest < 0)
                        now_to_latest += 8600;

                    String now_to_earliest_string = String.valueOf(now_to_earliest);
                    String now_to_latest_string = String.valueOf(now_to_latest);

                    if(now_to_earliest > now_to_latest)
                        status.setText("Earliest has to be before Latest Wake Up Time" + now_to_earliest_string + " : " + now_to_latest_string);
                    else if (connected){
                        mConnectedThread = new ConnectedThread(btSocket);
                        mConnectedThread.start();

                        String msg = "*"+now_to_earliest_string+"@"+now_to_latest_string+"$";
                        status.setText("Sending: " + msg + " over bt to " + btSocket.getRemoteDevice().getName());


                        mConnectedThread.write(msg);
                    }
                    else{
                        status.setText("device not connected");
                    }
                }
            });
            mEarliestTimeButton.setOnClickListener(new OnClickListener()
            {
                @Override
                public void onClick(View v)
                {
                    mTargetTimeText = mEarliestTimeText;
                    targetTime = earliestTime;
                    earliestTimeIsSet = true;
                    DialogFragment newFragment = new TimePickerFragment();
                    newFragment.show(getSupportFragmentManager(), "earliest time picker");
                }
            });
            mLatestTimeButton.setOnClickListener(new OnClickListener()
            {
                @Override
                public void onClick(View v)
                {
                    targetTime = latestTime;
                    mTargetTimeText = mLatestTimeText;
                    latestTimeIsSet = true;
                    DialogFragment newFragment = new TimePickerFragment();
                    newFragment.show(getSupportFragmentManager(), "latest time picker");
                }
            });
        }
    }

    public void on(View view){
        if (!myBluetoothAdapter.isEnabled()) {
            Intent turnOnIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(turnOnIntent, REQUEST_ENABLE_BT);

            Toast.makeText(getApplicationContext(),"Bluetooth turned on" ,
                    Toast.LENGTH_LONG).show();
        }
        else{
            Toast.makeText(getApplicationContext(),"Bluetooth is already on",
                    Toast.LENGTH_LONG).show();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if(requestCode == REQUEST_ENABLE_BT){
            if(myBluetoothAdapter.isEnabled()) {
                text.setText("Status: Enabled");
            } else {
                text.setText("Status: Disabled");
            }
        }
    }

    public void list(View view){
        // get paired devices
        pairedDevices = myBluetoothAdapter.getBondedDevices();

        // put it's one to the adapter
        for(BluetoothDevice device : pairedDevices)
            BTArrayAdapter.add(device.getName()+ "\n" + device.getAddress());

        Toast.makeText(getApplicationContext(),"Show Paired Devices",
                Toast.LENGTH_SHORT).show();

    }

    final BroadcastReceiver bReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            // When discovery finds a device
            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                // Get the BluetoothDevice object from the Intent
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                // add the name and the MAC address of the object to the arrayAdapter
                BTArrayAdapter.add(device.getName() + "\n" + device.getAddress());
                BTArrayAdapter.notifyDataSetChanged();

                if(isProperDevice(device)){
                    status.setText("Sleep Mask HC-06 Identified");
                    try {
                        btSocket = createBluetoothSocket(device);
                    } catch (IOException e) {
                        Toast.makeText(getBaseContext(), "Socket creation failed", Toast.LENGTH_LONG).show();
                    }
                    // Establish the Bluetooth socket connection.
                    try
                    {
                        btSocket.connect();
                        connected = true;
                        status.setText("connected with " + device.getAddress());
                    } catch (IOException e) {
                        try
                        {
                            btSocket.close();
                            status.setText("btsocket connect failure: "+device.getName());
                        } catch (IOException e2)
                        {
                            //insert code to deal with this
                        }
                    }
                }
                else{
                    status.setText(device.getName()+" Not Identified");
                }
            }
        }
    };

    public void find(View view) {
        status.setText("finding devices");

        if (myBluetoothAdapter.isDiscovering()) {
            status.setText("cancelling search");
            // the button is pressed when it discovers, so cancel the discovery
            myBluetoothAdapter.cancelDiscovery();
        }
        else {
            BTArrayAdapter.clear();
            String closed_msg = "";
            if(connected){
                try{
                    String name = btSocket.getRemoteDevice().getName();
                    btSocket.close();
                    connected = false;
                    closed_msg = "closing " + name +" .";
                } catch (IOException e){
                    status.setText("closing btsocked failed with "+btSocket.getRemoteDevice().getName());
                }
            }

            myBluetoothAdapter.startDiscovery();
            status.setText(closed_msg+"searching...");

            registerReceiver(bReceiver, new IntentFilter(BluetoothDevice.ACTION_FOUND));
        }
    }

    public void off(View view){
        myBluetoothAdapter.disable();
        text.setText("Status: Disconnected");

        Toast.makeText(getApplicationContext(),"Bluetooth turned off",
                Toast.LENGTH_LONG).show();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unregisterReceiver(bReceiver);
    }

    /*@Override
    public void onPause()
    {
        super.onPause();
        try
        {
            //Don't leave Bluetooth sockets open when leaving activity
            btSocket.close();
        } catch (IOException e2) {
            //insert code to deal with this
        }
    }*/

    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {
        return  device.createRfcommSocketToServiceRecord(BTMODULEUUID);
        //creates secure outgoing connecetion with BT device using UUID
    }

    private class ConnectedThread extends Thread {
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        //creation of the connect thread
        public ConnectedThread(BluetoothSocket socket) {
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                //Create I/O streams for connection
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) { }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }

        //write method
        public void write(String input) {
            byte[] msgBuffer = input.getBytes();           //converts entered String into bytes
            try {
                mmOutStream.write(msgBuffer);                //write bytes over BT connection via outstream
            } catch (IOException e) {
                //if you cannot write, close the application
                Toast.makeText(getBaseContext(), "Connection Failure", Toast.LENGTH_LONG).show();
                finish();

            }
        }
    }
}