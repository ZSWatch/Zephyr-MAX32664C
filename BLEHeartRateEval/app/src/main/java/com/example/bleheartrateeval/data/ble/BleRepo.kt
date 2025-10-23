package com.example.bleheartrateeval.data.ble

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.util.Log
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import no.nordicsemi.android.ble.PhyRequest
import no.nordicsemi.android.ble.observer.BondingObserver
import no.nordicsemi.android.ble.observer.ConnectionObserver
import java.util.UUID

class BleRepo(private val context: Context) {
    companion object { private const val TAG = "BleRepo" }
    private val scope = CoroutineScope(Dispatchers.Default + Job())

    private val bluetoothManager =
        context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val bluetoothAdapter: BluetoothAdapter? = bluetoothManager.adapter
    private val scanner: BluetoothLeScanner? get() = bluetoothAdapter?.bluetoothLeScanner

    private val _scanResults = MutableSharedFlow<ScanResult>(
        replay = 0, extraBufferCapacity = 64, onBufferOverflow = BufferOverflow.DROP_OLDEST
    )
    val scanResults: Flow<ScanResult> = _scanResults.asSharedFlow()

    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    val connectionState: Flow<ConnectionState> = _connectionState
    private val _linkStatus = MutableStateFlow(LinkStatus())
    val linkStatus: Flow<LinkStatus> = _linkStatus

    private val _isScanning = MutableStateFlow(false)
    val isScanning: Flow<Boolean> = _isScanning

    private val _lastRssi = MutableStateFlow<Int?>(null)
    val lastRssi: Flow<Int?> = _lastRssi

    private val _incoming = MutableSharedFlow<ByteArray>(
        replay = 0, extraBufferCapacity = 128, onBufferOverflow = BufferOverflow.DROP_OLDEST
    )
    val incoming: Flow<ByteArray> = _incoming.asSharedFlow()

    private var nusManager: NusManager? = null
    private var activeDevice: BluetoothDevice? = null
    private var rssiJob: Job? = null
    private val _connectedDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val connectedDevices = _connectedDevices.asStateFlow()

    private fun mutateLinkStatus(block: (LinkStatus) -> LinkStatus) {
        _linkStatus.value = block(_linkStatus.value)
    }

    private fun updateConnectionState(state: ConnectionState) {
        _connectionState.value = state
        mutateLinkStatus { it.copy(connectionState = state) }
    }

    private fun setBondStatus(status: BondStatus) {
        mutateLinkStatus { it.copy(bondStatus = status) }
    }

    private fun setMtuValue(mtu: Int?) {
        mutateLinkStatus { it.copy(mtu = mtu) }
    }

    private fun setPhyValue(tx: Int?, rx: Int?) {
        mutateLinkStatus { it.copy(txPhy = tx, rxPhy = rx) }
    }

    private fun bondStatusOf(device: BluetoothDevice): BondStatus = when (device.bondState) {
        BluetoothDevice.BOND_BONDED -> BondStatus.Bonded
        BluetoothDevice.BOND_BONDING -> BondStatus.Bonding
        BluetoothDevice.BOND_NONE -> BondStatus.NotBonded
        else -> BondStatus.Unknown
    }

    fun isBlePermissionGranted(): Boolean =
        BlePermissions.allGranted(context)

    @SuppressLint("MissingPermission")
    fun startScan(filterServiceUuid: UUID? = null, filterLocalName: String? = null) {
        if (!isBlePermissionGranted()) {
            Log.w(TAG, "BLE permissions not granted; scan aborted")
            _isScanning.value = false
            return
        }
        if (bluetoothAdapter?.isEnabled != true) {
            Log.w(TAG, "Bluetooth adapter disabled; scan aborted")
            _isScanning.value = false
            return
        }
        _lastRssi.value = null
        val filters = mutableListOf<ScanFilter>()
        if (filterServiceUuid != null) {
            filters.add(
                ScanFilter.Builder().setServiceUuid(android.os.ParcelUuid(filterServiceUuid)).build()
            )
        }
        if (!filterLocalName.isNullOrEmpty()) {
            filters.add(ScanFilter.Builder().setDeviceName(filterLocalName).build())
        }

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        Log.d(TAG, "Starting scan; filters=${filters.size} nameFilter=${filterLocalName}")
        val s = scanner
        if (s == null) {
            Log.e(TAG, "BluetoothLeScanner is null; cannot start scan")
            _isScanning.value = false
            return
        }
        try {
            s.startScan(filters.ifEmpty { null }, settings, scanCallback)
            _isScanning.value = true
        } catch (e: Exception) {
            _isScanning.value = false
            Log.e(TAG, "Failed to start scan", e)
        }
    }

    @SuppressLint("MissingPermission")
    fun stopScan() {
        if (!isBlePermissionGranted()) return
        scanner?.stopScan(scanCallback)
        _isScanning.value = false
    }

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            Log.d(TAG, "scanResult: ${result.device.address} name=${result.device.name} rssi=${result.rssi}")
            _scanResults.tryEmit(result)
            if (result.device.address == activeDevice?.address) {
                _lastRssi.value = result.rssi
            }
        }

        override fun onBatchScanResults(results: MutableList<ScanResult>) {
            Log.d(TAG, "batchResults: count=${results.size}")
            results.forEach { _scanResults.tryEmit(it) }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "scanFailed: code=$errorCode")
            _isScanning.value = false
        }
    }

    val currentConnectionState: ConnectionState get() = _connectionState.value
    val currentDeviceAddress: String? get() = activeDevice?.address

    init {
        refreshConnectedDevices()
    }

    fun refreshConnectedDevices() {
        updateConnectedDevices()
    }

    @SuppressLint("MissingPermission")
    private fun updateConnectedDevices() {
        if (!isBlePermissionGranted()) {
            _connectedDevices.value = emptyList()
            return
        }
        try {
            val devices = bluetoothManager.getConnectedDevices(BluetoothProfile.GATT)
            _connectedDevices.value = devices
        } catch (e: SecurityException) {
            Log.e(TAG, "updateConnectedDevices: missing permission", e)
            _connectedDevices.value = emptyList()
        } catch (e: Exception) {
            Log.e(TAG, "updateConnectedDevices: unexpected error", e)
        }
    }

    fun connect(device: BluetoothDevice, useAutoConnect: Boolean = false): Boolean {
        if (!isBlePermissionGranted()) return false

        Log.d(
            TAG,
            "connect(): device=${device.address} name=${device.name} autoConnect=$useAutoConnect " +
                "currentState=${_connectionState.value} activeDevice=${activeDevice?.address}"
        )

        val manager = NusManager(context)
        nusManager = manager
        activeDevice = device
        mutateLinkStatus {
            it.copy(
                bondStatus = bondStatusOf(device),
                mtu = null,
                txPhy = null,
                rxPhy = null
            )
        }
        manager.onMtuChanged = { mtu -> setMtuValue(mtu) }
        manager.onPhyChanged = { tx, rx -> setPhyValue(tx, rx) }

        // Stop scanning before initiating a connection to reduce radio contention
        try { stopScan() } catch (_: Exception) {}

        if (!useAutoConnect && device.bondState == BluetoothDevice.BOND_NONE) {
            val initiated = try {
                Log.d(TAG, "connect: initiating bond before GATT connection")
                device.createBond()
            } catch (e: SecurityException) {
                Log.e(TAG, "connect: createBond requires BLUETOOTH_CONNECT permission", e)
                false
            } catch (e: Exception) {
                Log.e(TAG, "connect: unexpected error creating bond", e)
                false
            }
            if (initiated) {
                setBondStatus(BondStatus.Bonding)
            } else {
                Log.w(TAG, "connect: createBond() returned false")
            }
        }

        manager.setConnectionObserver(object : ConnectionObserver {
            override fun onDeviceConnecting(device: BluetoothDevice) {
                Log.d(TAG, "onDeviceConnecting: ${device.address}")
                updateConnectionState(ConnectionState.Connecting)
            }

            override fun onDeviceConnected(device: BluetoothDevice) {
                Log.d(TAG, "onDeviceConnected: ${device.address}")
                updateConnectionState(ConnectionState.Connected)
                setBondStatus(bondStatusOf(device))
                updateConnectedDevices()
            }

            override fun onDeviceFailedToConnect(device: BluetoothDevice, reason: Int) {
                Log.e(TAG, "onDeviceFailedToConnect: reason=$reason")
                updateConnectionState(ConnectionState.Failed(reason))
                stopRssiMonitor()
                _lastRssi.value = null
            }

            override fun onDeviceReady(device: BluetoothDevice) {
                Log.d(TAG, "onDeviceReady: ${device.address}")
                updateConnectionState(ConnectionState.Ready)
                startRssiMonitor(manager)
                updateConnectedDevices()
            }

            override fun onDeviceDisconnecting(device: BluetoothDevice) {
                Log.d(TAG, "onDeviceDisconnecting: ${device.address}")
                updateConnectionState(ConnectionState.Disconnecting)
            }

            override fun onDeviceDisconnected(device: BluetoothDevice, reason: Int) {
                Log.d(TAG, "onDeviceDisconnected: ${device.address} reason=$reason")
                updateConnectionState(ConnectionState.Disconnected)
                setBondStatus(BondStatus.Unknown)
                setMtuValue(null)
                setPhyValue(null, null)
                stopRssiMonitor()
                _lastRssi.value = null
                updateConnectedDevices()
            }
        })

        manager.setBondingObserver(object : BondingObserver {
            override fun onBondingRequired(device: BluetoothDevice) {
                Log.d(TAG, "onBondingRequired: ${device.address}")
                setBondStatus(BondStatus.Bonding)
            }

            override fun onBonded(device: BluetoothDevice) {
                Log.d(TAG, "onBonded: ${device.address}")
                setBondStatus(BondStatus.Bonded)
            }

            override fun onBondingFailed(device: BluetoothDevice) {
                Log.w(TAG, "onBondingFailed: ${device.address}")
                setBondStatus(BondStatus.NotBonded)
            }
        })

        manager.onDataReceived = { bytes ->
            _incoming.tryEmit(bytes)
        }

        Log.d(
            TAG,
            "connect(): enqueue connect request autoConnect=$useAutoConnect bondState=${device.bondState}"
        )
        manager.connect(device)
            .useAutoConnect(useAutoConnect)
            .usePreferredPhy(PhyRequest.PHY_LE_2M_MASK or PhyRequest.PHY_LE_1M_MASK)
            .timeout(15_000)
            .retry(3, 100)
            .enqueue()
        return true
    }

    @SuppressLint("MissingPermission")
    fun connect(address: String, useAutoConnect: Boolean = false): Boolean {
        if (!isBlePermissionGranted()) return false
        val adapter = bluetoothAdapter
        if (adapter == null) {
            Log.e(TAG, "connect: BluetoothAdapter unavailable")
            return false
        }
        val device = try {
            adapter.getRemoteDevice(address)
        } catch (e: IllegalArgumentException) {
            Log.e(TAG, "connect: invalid device address=$address")
            return false
        }
        return connect(device, useAutoConnect)
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        if (!isBlePermissionGranted()) return
        Log.d(
            TAG,
            "disconnect(): requested activeDevice=${activeDevice?.address} " +
                "currentState=${_connectionState.value}"
        )
        try { stopScan() } catch (_: Exception) {}

        stopRssiMonitor()

        fun resetConnectionState() {
            Log.d(
                TAG,
                "resetConnectionState(): clearing repo state (activeDevice=${activeDevice?.address})"
            )
            nusManager = null
            activeDevice = null
            updateConnectionState(ConnectionState.Disconnected)
            setBondStatus(BondStatus.Unknown)
            setMtuValue(null)
            setPhyValue(null, null)
            _lastRssi.value = null
            updateConnectedDevices()
        }

        val manager = nusManager ?: run {
            Log.d(TAG, "disconnect(): no active NusManager; resetting state only")
            resetConnectionState()
            updateConnectedDevices()
            return
        }

        fun closeManager(tag: String) {
            try {
                manager.close()
                Log.d(TAG, "$tag: manager closed")
            } catch (e: Exception) {
                Log.e(TAG, "$tag: error closing manager", e)
            } finally {
                resetConnectionState()
            }
        }

        manager.cancelPendingOperations()
        Log.d(TAG, "disconnect(): pending operations cancelled")

        try {
            manager.disconnect()
                .done {
                Log.d(TAG, "disconnect(): success callback")
                closeManager("disconnect.done")
            }
                .fail { _, status ->
                    Log.w(TAG, "disconnect(): fail status=$status")
                    closeManager("disconnect.fail")
                }
                .enqueue()
        } catch (e: Exception) {
            Log.e(TAG, "disconnect(): unexpected exception", e)
            closeManager("disconnect.exception")
        }
    }

    private fun startRssiMonitor(manager: NusManager) {
        stopRssiMonitor()
        rssiJob = scope.launch {
            while (isActive) {
                try {
                    manager.readRssiOnce { rssi -> _lastRssi.value = rssi }
                } catch (e: Exception) {
                    Log.w(TAG, "readRssiOnce failed", e)
                }
                delay(4000L)
            }
        }
    }

    private fun stopRssiMonitor() {
        rssiJob?.cancel()
        rssiJob = null
    }
}

sealed class ConnectionState {
    data object Disconnected : ConnectionState()
    data object Connecting : ConnectionState()
    data object Connected : ConnectionState()
    data object Ready : ConnectionState()
    data object Disconnecting : ConnectionState()
    data class Failed(val reason: Int) : ConnectionState()
}

enum class BondStatus {
    Unknown,
    NotBonded,
    Bonding,
    Bonded
}

data class LinkStatus(
    val connectionState: ConnectionState = ConnectionState.Disconnected,
    val bondStatus: BondStatus = BondStatus.Unknown,
    val mtu: Int? = null,
    val txPhy: Int? = null,
    val rxPhy: Int? = null
)
