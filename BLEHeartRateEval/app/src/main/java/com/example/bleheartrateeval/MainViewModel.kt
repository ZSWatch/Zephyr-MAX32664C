package com.example.bleheartrateeval

import android.app.Application
import android.bluetooth.BluetoothDevice
import android.bluetooth.le.ScanResult
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.example.bleheartrateeval.data.LogWriter
import com.example.bleheartrateeval.data.Parser
import com.example.bleheartrateeval.data.ble.ConnectionState
import com.example.bleheartrateeval.data.ble.BleRepoHolder
import com.example.bleheartrateeval.data.ble.LinkStatus
import com.example.bleheartrateeval.data.model.Record
import com.example.bleheartrateeval.service.ForegroundReconnectService
import java.io.File
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

class MainViewModel(app: Application) : AndroidViewModel(app) {
    private val repo = BleRepoHolder.get(app)
    private val logger = LogWriter(app)

    private val _connectionState = MutableStateFlow<ConnectionState>(ConnectionState.Disconnected)
    val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private val _records = MutableStateFlow<List<Record>>(emptyList())
    val records: StateFlow<List<Record>> = _records.asStateFlow()

    private val _loggingEnabled = MutableStateFlow(false)
    val loggingEnabled: StateFlow<Boolean> = _loggingEnabled.asStateFlow()

    // Recent raw NUS lines (for UI display)
    private val _rawLines = MutableStateFlow<List<String>>(emptyList())
    val rawLines: StateFlow<List<String>> = _rawLines.asStateFlow()

    private val _lastRecord = MutableStateFlow<Record?>(null)
    val lastRecord: StateFlow<Record?> = _lastRecord.asStateFlow()

    private val _linkStatus = MutableStateFlow(LinkStatus())
    val linkStatus: StateFlow<LinkStatus> = _linkStatus.asStateFlow()

    private val _isScanning = MutableStateFlow(false)
    val isScanning: StateFlow<Boolean> = _isScanning.asStateFlow()

    private val _lastRssi = MutableStateFlow<Int?>(null)
    val lastRssi: StateFlow<Int?> = _lastRssi.asStateFlow()
    private val _connectedDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val connectedDevices: StateFlow<List<BluetoothDevice>> = _connectedDevices.asStateFlow()

    // Keep only 30 seconds of data for better performance
    private val bufferWindowMillis = 30 * 1000L

    init {
        // Observe connection state
        viewModelScope.launch {
            repo.connectionState.collectLatest { _connectionState.value = it }
        }
        viewModelScope.launch {
            repo.linkStatus.collectLatest { _linkStatus.value = it }
        }
        viewModelScope.launch {
            repo.isScanning.collectLatest { _isScanning.value = it }
        }
        viewModelScope.launch {
            repo.lastRssi.collectLatest { _lastRssi.value = it }
        }
        viewModelScope.launch {
            repo.connectedDevices.collectLatest { _connectedDevices.value = it }
        }
        // Observe incoming notifications and parse
        viewModelScope.launch {
            repo.incoming.collectLatest { bytes ->
                val ts = System.currentTimeMillis()
                val raw = bytes.toString(Charsets.UTF_8)
                val rec = Parser.parse(raw, ts)
                if (_loggingEnabled.value) logger.appendRaw(ts, raw)
                // Maintain a rolling 30-second buffer for performance
                val cutoff = ts - bufferWindowMillis
                val updated = _records.value.filter { it.timestamp >= cutoff } + rec
                _records.value = updated
                _lastRecord.value = rec

                // Keep minimal raw lines (not displayed, only for debugging)
                val line = "${java.time.Instant.ofEpochMilli(ts)} | $raw"
                val rawUpdated = (_rawLines.value + line).takeLast(20)
                _rawLines.value = rawUpdated
            }
        }
    }

    val scanResults = repo.scanResults

    fun startScan(filterName: String? = null) {
        repo.refreshConnectedDevices()
        repo.startScan()
    }
    fun stopScan() = repo.stopScan()
    fun connect(result: ScanResult) {
        _lastRssi.value = result.rssi
        android.util.Log.d(
            "MainViewModel",
            "connect(): starting ForegroundReconnectService device=${result.device.address}"
        )
        ForegroundReconnectService.start(getApplication(), result.device, connectImmediately = false)
        repo.connect(result.device, useAutoConnect = false)
    }

    fun connectDevice(device: BluetoothDevice) {
        android.util.Log.d(
            "MainViewModel",
            "connectDevice(): adopting existing device=${device.address}"
        )
        _lastRssi.value = null
        ForegroundReconnectService.start(getApplication(), device, connectImmediately = false)
        repo.connect(device, useAutoConnect = false)
    }

    fun disconnect() {
        android.util.Log.d("MainViewModel", "disconnect(): stopping ForegroundReconnectService")
        ForegroundReconnectService.stop(getApplication(), disconnect = false)
        repo.disconnect()
        _lastRssi.value = null
    }

    fun enableLogging(overwrite: Boolean) {
        if (overwrite) {
            logger.clearToday()
        }
        _loggingEnabled.value = true
    }

    fun disableLogging() {
        _loggingEnabled.value = false
    }

    fun currentLogFile(): File? {
        val file = logger.todayFile()
        return if (file.exists() && file.length() > 0) file else null
    }

    fun hasLogFile(): Boolean = currentLogFile() != null

    fun clearRecords() {
        _records.value = emptyList()
        _lastRecord.value = null
    }
}
