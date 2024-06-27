package com.example.androidapp
import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Intent
import android.content.pm.ActivityInfo
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.util.Log
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.util.UUID
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

class MainActivity : AppCompatActivity() {

    private val REQUEST_ENABLE_BT = 1
    private val REQUEST_FINE_LOCATION_PERMISSION = 2
    private val REQUEST_BLUETOOTH_SCAN_PERMISSION = 3
    private val REQUEST_BLUETOOTH_CONNECT_PERMISSION = 4
    private var targetDevice: BluetoothDevice? = null // Add this variable to hold the target device
    private val POLLING_INTERVAL = 10 //Reading time interval

    val targetMacAddress = "54:F8:2A:50:68:2A" //For scanning using macAddress
    private val SUUID = "4906276B-DA6A-4A6C-BF94-73C61B96433C" // Replace with the OBU service UUID
    private val CUUID = "49AF5250-F176-46C5-B99A-A163A672C042" // Replace with the OBU characteristic UUID


    //initialise UI
    private lateinit var bluetoothAdapter: BluetoothAdapter
    private lateinit var bluetoothGatt: BluetoothGatt
    private lateinit var textView: TextView
    private lateinit var textView1: TextView
    private lateinit var textView2: TextView
    private lateinit var textView3: TextView
    private lateinit var textView4: TextView
    private lateinit var textView5: TextView
    private lateinit var textView6: TextView
    private lateinit var textView7: TextView
    private lateinit var textView8: TextView
    private lateinit var textView9: TextView
    private lateinit var textView10: TextView
    private lateinit var buttonConnect: Button
    private lateinit var buttonDisconnect: Button
    private var lastHighlightedTextView: TextView? = null

    @SuppressLint("MissingPermission")
    override fun onCreate(savedInstanceState: Bundle?) {
        setTheme(R.style.Theme_AppCompat)
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        textView = findViewById(R.id.textView)
        textView1 = findViewById(R.id.textView1)
        textView2 = findViewById(R.id.textView2)
        textView3 = findViewById(R.id.textView3)
        textView4 = findViewById(R.id.textView4)
        textView5 = findViewById(R.id.textView5)
        textView6 = findViewById(R.id.textView6)
        textView7 = findViewById(R.id.textView7)
        textView8 = findViewById(R.id.textView8)
        textView9 = findViewById(R.id.textView9)
        textView10 = findViewById(R.id.textView10)
        val buttonOK: Button = findViewById(R.id.buttonOK)

        buttonOK.setOnClickListener {
            resetHighlighting()
            lastHighlightedTextView?.setBackgroundResource(R.drawable.border)
        }

        buttonConnect = findViewById(R.id.buttonConnect)
        buttonDisconnect = findViewById(R.id.buttonDisconnect)

// Set click listeners for the Connect and Disconnect buttons
        buttonConnect.setOnClickListener {
            targetDevice?.let { it -> connectToDevice(it) }
        }

        buttonDisconnect.setOnClickListener {
            bluetoothGatt.disconnect()
        }

//portrait mode
        requestedOrientation = ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE

//initialise bluetoothAdapter
        bluetoothAdapter = (getSystemService(BLUETOOTH_SERVICE) as BluetoothManager).adapter

// Check if BLE is supported on the device
        if (!packageManager.hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            showToast("BLE is not supported on this device.")
            Log.e(TAG, "BLE is not supported on this device.")
            finish()
        }
//if bluetoothAdapter is null
        if (bluetoothAdapter == null || bluetoothAdapter.bluetoothLeScanner == null) {
            showToast("Bluetooth is not available on this device.")
            Log.e(TAG, "Bluetooth is not available on this device.")
            finish()
            return
        }
// Check if Bluetooth is enabled, and request the user to enable it if not
        if (!bluetoothAdapter.isEnabled) {
            val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT)
        } else {
            checkLocationPermission()
        }
    }

    //remove the highlight
    private fun resetHighlighting() {
// Reset the background color of the lastHighlightedTextView
        lastHighlightedTextView?.setBackgroundResource(R.drawable.border)
        lastHighlightedTextView?.setBackgroundResource(0)
        lastHighlightedTextView?.setTextColor(Color.WHITE)
    }

    //check permissions
    private fun checkLocationPermission() {
// Check if the app has location permission, and request it if not
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED
        ) {
            ActivityCompat.requestPermissions(
                this,
                arrayOf(Manifest.permission.ACCESS_FINE_LOCATION),
                REQUEST_FINE_LOCATION_PERMISSION
            )
        }
        else {
//For scanning using macAddress
            startScanning(targetMacAddress)

        }
    }

    //display message
    private fun showToast(message: String) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }
    //request permission
    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        when (requestCode) {
            REQUEST_FINE_LOCATION_PERMISSION -> {
                if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
// Location permission granted.
                    startScanning(targetMacAddress)
                } else {
                    showToast("Location permission denied. Your app may not work correctly.")
                }
            }
        }
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
    }


//For scanning using macAddress
//filter scan using mac address

    @SuppressLint("MissingPermission")
    private fun startScanning(targetMacAddress: String) {

        val scanFilters = listOf(
            ScanFilter.Builder()
                .setDeviceAddress(targetMacAddress)
                .build()
        )
        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()
        bluetoothAdapter.bluetoothLeScanner.startScan(
            scanFilters,
            scanSettings,
            scanCallback
        )
        showToast("Scanning for device with MAC address $targetMacAddress...")
        Log.d(TAG, "Scanning for device with MAC address $targetMacAddress...")

    }
    //after scanning
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            result?.device?.let {
                val deviceMacAddress = it.address
                if (deviceMacAddress == targetMacAddress) {
                    stopScanning()
                    targetDevice = it
                    buttonConnect.setOnClickListener {
                        connectToDevice(targetDevice!!)
                    }
                }
            }
        }
        //on scan fail
        override fun onScanFailed(errorCode: Int) {
            showToast("Scan failed with error code: $errorCode")
            Log.e(TAG, "Scan failed with error code: $errorCode")
        }
    }
    //stop scan and perform operation in scancallback
    @SuppressLint("MissingPermission")
    private fun stopScanning() {
        bluetoothAdapter.bluetoothLeScanner.stopScan(scanCallback)
        Log.d(TAG, "Scan stopped.")

    }
    //connection
    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice) {
        bluetoothGatt = device.connectGatt(this, false, gattCallback)
        showToast("Connecting to OBU.")
        Log.d(TAG, "Connecting to OBU...")

    }
    //gatt server communication
    private val gattCallback = object : BluetoothGattCallback() {
        private val RECONNECT_DELAY_MS = 1000 // 1 seconds delay
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
            runOnUiThread {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    showToast("Connected to OBU.")
                    Log.d(TAG, "Connected to OBU.")
                    gatt?.discoverServices() //discover services provided by obu
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    showToast("Disconnected from OBU.")
                    Log.e(TAG, "Disconnected from OBU.")
// Attempt to auto reconnect with a time delay
                    val device = targetDevice // Replace with the stored BluetoothDevice
                    device?.let {
                        Handler().postDelayed({
                            connectToDevice(it)
                        }, RECONNECT_DELAY_MS.toLong())
                    }


                }
            }
        }
        //search for uuid
        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt?, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt?.getService(UUID.fromString(SUUID))
                val characteristic =
                    service?.getCharacteristic(UUID.fromString(CUUID))
                characteristic?.let {
                    startPolling(gatt, characteristic)
                }
            } else {
                Log.e(TAG, "Service discovery failed with status: $status")
            }
        }
        //read periodically
        @SuppressLint("MissingPermission")
        private fun startPolling(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
// Create a scheduled executor to read the data periodically
            val executor = Executors.newSingleThreadScheduledExecutor()
            executor.scheduleAtFixedRate(
                {
// Read the characteristic value
                    gatt.readCharacteristic(characteristic)
                },
                0, POLLING_INTERVAL.toLong(), TimeUnit.MILLISECONDS
            )
        }
        //when read,retrieve value and display
        override fun onCharacteristicRead(
            gatt: BluetoothGatt?,
            characteristic: BluetoothGattCharacteristic?,
            status: Int
        ) {
            if (status == 0) {
                val data = characteristic?.value
                data?.let {
                    val receivedData = data.toString(Charsets.UTF_8)
// Display the received data in your app's UI
                    runOnUiThread {
                        if (receivedData == "0") {
                            resetHighlighting()
                            lastHighlightedTextView?.setBackgroundResource(R.drawable.highlight_border)
                        } else {
                            textView.text = receivedData
                            Log.d(TAG, "Received data: $receivedData")
                            highlightTextView(receivedData)
                        }
                    }
                }
            } else {
                Log.e(TAG, "Characteristic read failed with status: $status")
            }
        }
    }
    //highlight text
    private fun highlightTextView(receivedData: String) {
// Reset the previous highlighted TextView, if any
        lastHighlightedTextView?.setBackgroundResource(R.drawable.highlight_border)
// lastHighlightedTextView?.setBackgroundResource(0)
        lastHighlightedTextView?.setTextColor(Color.WHITE)

// Get the corresponding TextView based on the received data
        val textView = when (receivedData.toIntOrNull()) {
            1 -> findViewById<TextView>(R.id.textView1)
// 2 -> findViewById<TextView>(R.id.textView2)
//3 -> findViewById<TextView>(R.id.textView3)
            2 -> findViewById<TextView>(R.id.textView4)
            3 -> findViewById<TextView>(R.id.textView5)
            5 -> findViewById<TextView>(R.id.textView6)
            6 -> findViewById<TextView>(R.id.textView7)
            7 -> findViewById<TextView>(R.id.textView8)
            4 -> findViewById<TextView>(R.id.textView9)
            8 -> findViewById<TextView>(R.id.textView10)
            else -> null
        }

        textView?.apply {
            setBackgroundResource(R.drawable.highlight_border)
            setBackgroundColor(Color.RED) // You can use any color you prefer
            setTextColor(Color.WHITE)
        }

// Update the lastHighlightedTextView reference
        lastHighlightedTextView = textView
    }


    //Tag is replaced with BluetoothApp in the log
    companion object {
        const val TAG = "BluetoothApp"
    }
}

