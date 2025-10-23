package com.example.bleheartrateeval.service

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.bluetooth.BluetoothDevice
import android.content.Context
import android.content.Intent
import android.os.Build
import android.os.IBinder
import android.util.Log
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import androidx.core.content.ContextCompat
import com.example.bleheartrateeval.MainActivity
import com.example.bleheartrateeval.data.ble.BleRepoHolder
import com.example.bleheartrateeval.data.ble.ConnectionState
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.launch

/**
 * Foreground service that keeps the BLE session alive with exponential back-off reconnects.
 */
class ForegroundReconnectService : Service() {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)
    private val repo by lazy { BleRepoHolder.get(applicationContext) }
    private val notificationManager by lazy { NotificationManagerCompat.from(this) }

    private var observeJob: Job? = null
    private var reconnectJob: Job? = null

    private var targetAddress: String? = null
    private var targetName: String? = null
    private var shouldReconnect: Boolean = false
    private var currentBackoffMs: Long = INITIAL_BACKOFF_MS

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        createNotificationChannel()
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        when (intent?.action) {
            ACTION_START -> handleStart(intent)
            ACTION_STOP -> {
                val disconnect = intent.getBooleanExtra(EXTRA_DISCONNECT, false)
                handleStop(disconnect)
            }
            else -> {
                if (targetAddress == null) {
                    Log.w(TAG, "Service started without target; stopping")
                    stopForeground(STOP_FOREGROUND_REMOVE)
                    stopSelf()
                }
            }
        }
        return START_STICKY
    }

    private fun handleStart(intent: Intent) {
        val address = intent.getStringExtra(EXTRA_DEVICE_ADDRESS)
        if (address.isNullOrEmpty()) {
            Log.e(TAG, "handleStart: missing device address")
            stopSelf()
            return
        }
        Log.d(
            TAG,
            "handleStart: address=$address connectImmediately=${intent.getBooleanExtra(EXTRA_CONNECT_IMMEDIATELY, true)}"
        )
        targetAddress = address
        targetName = intent.getStringExtra(EXTRA_DEVICE_NAME)
        shouldReconnect = true
        currentBackoffMs = INITIAL_BACKOFF_MS
        val connectImmediately = intent.getBooleanExtra(EXTRA_CONNECT_IMMEDIATELY, true)

        val initialNotification = buildNotification(
            status = "Connecting...",
            detail = null,
            ongoing = true
        )
        startForeground(NOTIFICATION_ID, initialNotification)
        observeConnectionStates()

        val state = repo.currentConnectionState
        val alreadyTargeted = repo.currentDeviceAddress == address &&
                state !is ConnectionState.Disconnected &&
                state !is ConnectionState.Failed

        if (connectImmediately) {
            Log.d(TAG, "handleStart: connectImmediately; alreadyTargeted=$alreadyTargeted")
            if (!alreadyTargeted) {
                val success = repo.connect(address, useAutoConnect = true)
                if (!success) {
                    scheduleReconnect("Connect request rejected")
                }
            } else {
                updateNotificationForState(state)
            }
        } else {
            updateNotificationForState(state)
        }
    }

    private fun handleStop(disconnect: Boolean) {
        Log.d(TAG, "handleStop: disconnect=$disconnect")
        shouldReconnect = false
        targetAddress = null
        targetName = null
        reconnectJob?.cancel()
        reconnectJob = null
        observeJob?.cancel()
        observeJob = null
        if (disconnect) {
            if (repo.currentDeviceAddress != null) {
                Log.d(TAG, "handleStop: disconnect flag set; repo will disconnect active device")
                repo.disconnect()
            } else {
                Log.d(TAG, "handleStop: disconnect flag set but no active device; skipping disconnect()")
            }
        }
        stopForeground(STOP_FOREGROUND_REMOVE)
        stopSelf()
    }

    private fun observeConnectionStates() {
        if (observeJob != null) return
        observeJob = scope.launch {
            repo.connectionState.collectLatest { state ->
                updateNotificationForState(state)
                when (state) {
                    ConnectionState.Disconnected -> scheduleReconnect("Link lost")
                    is ConnectionState.Failed -> scheduleReconnect("Failed (${state.reason})")
                    ConnectionState.Connecting -> cancelReconnectJob()
                    ConnectionState.Connected, ConnectionState.Ready -> {
                        cancelReconnectJob()
                        currentBackoffMs = INITIAL_BACKOFF_MS
                    }
                    ConnectionState.Disconnecting -> cancelReconnectJob()
                }
            }
        }
    }

    private fun scheduleReconnect(reason: String) {
        if (!shouldReconnect) return
        val address = targetAddress ?: return

        val delayMs = currentBackoffMs
        Log.d(TAG, "scheduleReconnect: reason=$reason delayMs=$delayMs address=$address")
        cancelReconnectJob()
        updateNotification(
            status = "Reconnecting in ${delayMs / 1000}s",
            detail = reason
        )

        reconnectJob = scope.launch {
            delay(delayMs)
            if (!shouldReconnect) return@launch
            Log.d(TAG, "scheduleReconnect: attempting reconnect address=$address")
            val success = repo.connect(address, useAutoConnect = true)
            if (!success) {
                Log.w(TAG, "scheduleReconnect: connect() call rejected; will retry")
            }
            currentBackoffMs = (currentBackoffMs * 2).coerceAtMost(MAX_BACKOFF_MS)
        }
    }

    private fun cancelReconnectJob() {
        if (reconnectJob != null) {
            Log.d(TAG, "cancelReconnectJob")
        }
        reconnectJob?.cancel()
        reconnectJob = null
    }

    private fun updateNotificationForState(state: ConnectionState) {
        val status = when (state) {
            ConnectionState.Connecting -> "Connecting..."
            ConnectionState.Connected -> "Connected (initializing)"
            ConnectionState.Ready -> "Streaming data"
            ConnectionState.Disconnecting -> "Disconnecting..."
            ConnectionState.Disconnected -> "Disconnected"
            is ConnectionState.Failed -> "Failed (${state.reason})"
        }
        updateNotification(status, detail = null)
    }

    private fun updateNotification(status: String, detail: String?) {
        val ongoing = shouldReconnect
        val notification = buildNotification(status, detail, ongoing)
        notificationManager.notify(NOTIFICATION_ID, notification)
    }

    private fun buildNotification(status: String, detail: String?, ongoing: Boolean): Notification {
        val title = targetName?.takeIf { it.isNotBlank() }
            ?: targetAddress?.let { "BLE session $it" }
            ?: "BLE session"

        val contentIntent = PendingIntent.getActivity(
            this,
            0,
            Intent(this, MainActivity::class.java).apply {
                flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP
            },
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
        )
        val stopIntent = PendingIntent.getService(
            this,
            1,
            Intent(this, ForegroundReconnectService::class.java).apply {
                action = ACTION_STOP
                putExtra(EXTRA_DISCONNECT, true)
            },
            PendingIntent.FLAG_IMMUTABLE
        )

        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setOngoing(ongoing)
            .setSmallIcon(android.R.drawable.stat_sys_data_bluetooth)
            .setContentTitle(title)
            .setContentText(status)
            .setContentIntent(contentIntent)
            .setPriority(NotificationCompat.PRIORITY_LOW)
            .setCategory(NotificationCompat.CATEGORY_SERVICE)
            .addAction(
                android.R.drawable.ic_media_pause,
                "Stop",
                stopIntent
            )

        if (!detail.isNullOrBlank()) {
            builder.setStyle(
                NotificationCompat.BigTextStyle()
                    .bigText("$status\n$detail")
            )
        }

        return builder.build()
    }

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O) return
        val channel = NotificationChannel(
            CHANNEL_ID,
            "BLE Session",
            NotificationManager.IMPORTANCE_LOW
        ).apply {
            description = "Keeps BLE session alive for auto reconnect"
        }
        val nm = getSystemService(NotificationManager::class.java)
        nm?.createNotificationChannel(channel)
    }

    override fun onDestroy() {
        super.onDestroy()
        scope.cancel()
    }

    companion object {
        private const val TAG = "ForegroundReconnectSvc"
        private const val CHANNEL_ID = "ble_session_channel"
        private const val NOTIFICATION_ID = 42
        private const val INITIAL_BACKOFF_MS = 1_000L
        private const val MAX_BACKOFF_MS = 8_000L

        private const val ACTION_START = "com.example.bleheartrateeval.service.action.START"
        private const val ACTION_STOP = "com.example.bleheartrateeval.service.action.STOP"

        private const val EXTRA_DEVICE_ADDRESS = "extra_device_address"
        private const val EXTRA_DEVICE_NAME = "extra_device_name"
        private const val EXTRA_DISCONNECT = "extra_disconnect"
        private const val EXTRA_CONNECT_IMMEDIATELY = "extra_connect_now"

        fun start(context: Context, device: BluetoothDevice, connectImmediately: Boolean = true) {
            val intent = Intent(context, ForegroundReconnectService::class.java).apply {
                action = ACTION_START
                putExtra(EXTRA_DEVICE_ADDRESS, device.address)
                putExtra(EXTRA_DEVICE_NAME, device.name)
                putExtra(EXTRA_CONNECT_IMMEDIATELY, connectImmediately)
            }
            ContextCompat.startForegroundService(context, intent)
        }

        fun start(
            context: Context,
            address: String,
            name: String? = null,
            connectImmediately: Boolean = true
        ) {
            val intent = Intent(context, ForegroundReconnectService::class.java).apply {
                action = ACTION_START
                putExtra(EXTRA_DEVICE_ADDRESS, address)
                putExtra(EXTRA_DEVICE_NAME, name)
                putExtra(EXTRA_CONNECT_IMMEDIATELY, connectImmediately)
            }
            ContextCompat.startForegroundService(context, intent)
        }

        fun stop(context: Context, disconnect: Boolean = false) {
            val intent = Intent(context, ForegroundReconnectService::class.java).apply {
                action = ACTION_STOP
                putExtra(EXTRA_DISCONNECT, disconnect)
            }
            context.startService(intent)
        }
    }
}
