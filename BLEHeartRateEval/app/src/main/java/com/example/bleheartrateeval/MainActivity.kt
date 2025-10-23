package com.example.bleheartrateeval

import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.WindowInsets
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.systemBars
import androidx.compose.foundation.layout.windowInsetsPadding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.MaterialTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.SideEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.luminance
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalView
import androidx.core.app.ActivityCompat
import androidx.core.content.FileProvider
import androidx.core.view.WindowCompat
import androidx.core.view.WindowInsetsControllerCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import androidx.lifecycle.compose.LocalLifecycleOwner
import com.example.bleheartrateeval.data.ble.BlePermissions
import com.example.bleheartrateeval.data.ble.ConnectionState
import com.example.bleheartrateeval.ui.ScanScreen
import com.example.bleheartrateeval.ui.SessionScreen
import com.example.bleheartrateeval.ui.theme.BLEHeartRateEvalTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        
        // Configure window to draw edge-to-edge
        WindowCompat.setDecorFitsSystemWindows(window, false)
        
        setContent {
            BLEHeartRateEvalTheme {
                val view = LocalView.current
                val colors = MaterialTheme.colorScheme
                SideEffect {
                    val window = (view.context as Activity).window
                    val statusColor = colors.background.toArgb()
                    window.statusBarColor = statusColor
                    window.navigationBarColor = statusColor
                    WindowInsetsControllerCompat(window, view).apply {
                        val useLightIcons = colors.background.luminance() < 0.5f
                        isAppearanceLightStatusBars = !useLightIcons
                        isAppearanceLightNavigationBars = !useLightIcons
                    }
                }
                
                Scaffold(
                    modifier = Modifier
                        .fillMaxSize()
                        .background(colors.background)
                        .windowInsetsPadding(WindowInsets.systemBars),
                    containerColor = colors.background
                ) { innerPadding ->
                    App(modifier = Modifier.padding(innerPadding))
                }
            }
        }
    }
}

@Composable
fun App(modifier: Modifier = Modifier) {
    val vm: MainViewModel = viewModel()
    val context = LocalContext.current
    val activity = context as? Activity
    val requiredPermissions = remember { BlePermissions.requiredPermissions() }
    var permissionRefresh by remember { mutableStateOf(0) }
    var hasRequestedPermissions by remember { mutableStateOf(false) }
    val permissionLauncher = rememberLauncherForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) {
        permissionRefresh++
    }
    val missingPermissions = remember(permissionRefresh, context) {
        BlePermissions.missingPermissions(context)
    }
    val permissionsGranted = missingPermissions.isEmpty()
    val shouldShowPermissionRationale = missingPermissions.any { permission ->
        activity?.let { ActivityCompat.shouldShowRequestPermissionRationale(it, permission) } == true
    }
    val requestPermissions: () -> Unit = remember(permissionLauncher, requiredPermissions) {
        {
            hasRequestedPermissions = true
            permissionLauncher.launch(requiredPermissions)
        }
    }
    val openAppSettings: () -> Unit = remember(context) {
        {
            val intent = Intent(
                android.provider.Settings.ACTION_APPLICATION_DETAILS_SETTINGS,
                Uri.fromParts("package", context.packageName, null)
            ).apply {
                addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            context.startActivity(intent)
        }
    }
    val lifecycleOwner = LocalLifecycleOwner.current
    DisposableEffect(lifecycleOwner) {
        val observer = LifecycleEventObserver { _, event ->
            if (event == Lifecycle.Event.ON_RESUME) {
                permissionRefresh++
            }
        }
        lifecycleOwner.lifecycle.addObserver(observer)
        onDispose { lifecycleOwner.lifecycle.removeObserver(observer) }
    }

    val connectionState by vm.connectionState.collectAsState()
    val records by vm.records.collectAsState()
    val logging by vm.loggingEnabled.collectAsState()
    val rawLines by vm.rawLines.collectAsState()
    val linkStatus by vm.linkStatus.collectAsState()
    val isScanning by vm.isScanning.collectAsState()
    val lastRssi by vm.lastRssi.collectAsState()
    val connectedDevices by vm.connectedDevices.collectAsState()
    val canShareLog = vm.hasLogFile()
    val shareLog: () -> Unit = remember(context, vm, canShareLog) {
        {
            val file = vm.currentLogFile()
            if (file != null) {
                val uri = FileProvider.getUriForFile(
                    context,
                    "${context.packageName}.fileprovider",
                    file
                )
                val shareIntent = Intent(Intent.ACTION_SEND).apply {
                    type = "text/plain"
                    putExtra(Intent.EXTRA_STREAM, uri)
                    putExtra(Intent.EXTRA_SUBJECT, "BLE log ${file.name}")
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                }
                val chooser = Intent.createChooser(shareIntent, "Share log")
                context.startActivity(chooser)
            } else {
                Toast.makeText(context, "No log file available", Toast.LENGTH_SHORT).show()
            }
        }
    }

    Box(modifier = modifier.fillMaxSize()) {
        when (connectionState) {
            is ConnectionState.Disconnected, is ConnectionState.Failed -> {
                ScanScreen(
                    results = vm.scanResults,
                    connectedDevices = connectedDevices,
                    permissionsGranted = permissionsGranted,
                    hasRequestedPermissions = hasRequestedPermissions,
                    missingPermissions = missingPermissions,
                    shouldShowPermissionRationale = shouldShowPermissionRationale,
                    isScanning = isScanning,
                    onRequestPermissions = requestPermissions,
                    onOpenSettings = openAppSettings,
                    onToggleScan = {
                        if (permissionsGranted) {
                            if (isScanning) vm.stopScan() else vm.startScan()
                        } else {
                            requestPermissions()
                        }
                    },
                    onConnect = { result ->
                        if (permissionsGranted) vm.connect(result) else requestPermissions()
                    },
                    onSelectConnected = { device ->
                        if (permissionsGranted) vm.connectDevice(device) else requestPermissions()
                    }
                )
            }
            else -> {
                SessionScreen(
                    linkStatus = linkStatus,
                    records = records,
                    loggingEnabled = logging,
                    onEnableLogging = vm::enableLogging,
                    onDisableLogging = vm::disableLogging,
                    lastRssi = lastRssi,
                    canShareLog = canShareLog,
                    onShareLog = shareLog,
                    onDisconnect = vm::disconnect,
                    onClearData = vm::clearRecords,
                    rawLines = rawLines
                )
            }
        }
    }
}
