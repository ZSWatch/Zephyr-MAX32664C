package com.example.bleheartrateeval.ui

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.bluetooth.le.ScanResult
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ElevatedButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import kotlinx.coroutines.flow.Flow

@Composable
fun ScanScreen(
    results: Flow<ScanResult>,
    connectedDevices: List<BluetoothDevice>,
    permissionsGranted: Boolean,
    hasRequestedPermissions: Boolean,
    missingPermissions: List<String>,
    shouldShowPermissionRationale: Boolean,
    isScanning: Boolean,
    onRequestPermissions: () -> Unit,
    onOpenSettings: () -> Unit,
    onToggleScan: () -> Unit,
    onConnect: (ScanResult) -> Unit,
    onSelectConnected: (BluetoothDevice) -> Unit,
) {
    var filter by rememberSaveable { mutableStateOf("Heartrate") }
    val list = remember { mutableStateListOf<ScanResult>() }

    LaunchedEffect(permissionsGranted) {
        list.clear()
        if (!permissionsGranted) return@LaunchedEffect
        results.collect { r ->
            val idx = list.indexOfFirst { it.device.address == r.device.address }
            if (idx >= 0) list[idx] = r else list.add(r)
            list.sortByDescending { it.rssi }
        }
    }

    Column(
        Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 20.dp, vertical = 24.dp)
    ) {
        if (!permissionsGranted) {
            PermissionRequestCard(
                missingPermissions = missingPermissions,
                hasRequestedPermissions = hasRequestedPermissions,
                shouldShowPermissionRationale = shouldShowPermissionRationale,
                onRequestPermissions = onRequestPermissions,
                onOpenSettings = onOpenSettings
            )
            return@Column
        }

        Text(
            text = "ZSWatch Heartrate Test",
            style = MaterialTheme.typography.headlineSmall,
            fontWeight = FontWeight.SemiBold,
            color = MaterialTheme.colorScheme.onBackground
        )
        Spacer(Modifier.height(4.dp))
        Text(
            text = "Scan for nearby prototypes and tap to connect.",
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurfaceVariant
        )
        Spacer(Modifier.height(20.dp))

        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            OutlinedTextField(
                value = filter,
                onValueChange = { filter = it },
                label = { Text("Device name filter") },
                placeholder = { Text("Heartrate") },
                singleLine = true,
                modifier = Modifier.weight(1f)
            )
            TextButton(onClick = { filter = "Heartrate" }) {
                Text("Reset")
            }
        }
        Spacer(Modifier.height(20.dp))

        ElevatedButton(
            onClick = onToggleScan,
            modifier = Modifier.fillMaxWidth(),
            contentPadding = PaddingValues(vertical = 14.dp)
        ) {
            val label = if (isScanning) "Stop scan" else "Start scan"
            Text(label)
        }

        val query = filter.trim()
        val filtered = if (query.isEmpty()) list.toList() else list.filter { result ->
            val nameMatch = result.device.name?.contains(query, ignoreCase = true) ?: false
            val addressMatch = result.device.address.contains(query, ignoreCase = true)
            nameMatch || addressMatch
        }

        Spacer(Modifier.height(16.dp))

        LazyColumn(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            if (connectedDevices.isNotEmpty()) {
                item {
                    SectionHeader(title = "Currently connected devices")
                }
                items(connectedDevices, key = { it.address }) { device ->
                    ConnectedDeviceCard(
                        device = device,
                        onSelect = { onSelectConnected(device) }
                    )
                }
                item { Spacer(Modifier.height(8.dp)) }
            }

            item {
                SectionHeader(title = "Scan results")
            }

            if (filtered.isEmpty()) {
                item {
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(vertical = 32.dp),
                        contentAlignment = Alignment.Center
                    ) {
                        Text(
                            text = if (isScanning) "Scanning..." else "No devices yet. Start a scan to discover watches.",
                            color = MaterialTheme.colorScheme.onSurfaceVariant
                        )
                    }
                }
            } else {
                items(filtered, key = { it.device.address }) { item ->
                    DeviceCard(
                        result = item,
                        onConnect = { onConnect(item) }
                    )
                }
            }
        }
    }
}

@Composable
private fun DeviceCard(
    result: ScanResult,
    onConnect: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        elevation = CardDefaults.cardElevation(defaultElevation = 1.dp)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 16.dp, vertical = 10.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(2.dp)
            ) {
                Text(
                    text = result.device.name ?: "Unnamed device",
                    style = MaterialTheme.typography.titleMedium,
                    color = MaterialTheme.colorScheme.onSurface,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
                Text(
                    text = result.device.address,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            Column(
                horizontalAlignment = Alignment.End,
                verticalArrangement = Arrangement.spacedBy(4.dp)
            ) {
                Text(
                    text = "${result.rssi} dBm",
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
                TextButton(
                    onClick = onConnect,
                    contentPadding = PaddingValues(horizontal = 12.dp, vertical = 2.dp)
                ) {
                    Text("Connect")
                }
            }
        }
    }
}

@Composable
private fun ConnectedDeviceCard(
    device: BluetoothDevice,
    onSelect: () -> Unit
) {
    Card(
        modifier = Modifier.fillMaxWidth(),
        colors = CardDefaults.cardColors(containerColor = MaterialTheme.colorScheme.surfaceVariant),
        elevation = CardDefaults.cardElevation(defaultElevation = 1.dp)
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(horizontal = 16.dp, vertical = 10.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column(
                modifier = Modifier.weight(1f),
                verticalArrangement = Arrangement.spacedBy(2.dp)
            ) {
                Text(
                    text = device.name ?: "Unnamed device",
                    style = MaterialTheme.typography.titleMedium,
                    color = MaterialTheme.colorScheme.onSurface,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
                Text(
                    text = device.address,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
            TextButton(
                onClick = onSelect,
                contentPadding = PaddingValues(horizontal = 12.dp, vertical = 2.dp)
            ) {
                Text("Connect")
            }
        }
    }
}

@Composable
private fun SectionHeader(title: String) {
    Text(
        text = title,
        style = MaterialTheme.typography.titleSmall,
        fontWeight = FontWeight.SemiBold,
        color = MaterialTheme.colorScheme.onSurfaceVariant
    )
}

@Composable
private fun PermissionRequestCard(
    missingPermissions: List<String>,
    hasRequestedPermissions: Boolean,
    shouldShowPermissionRationale: Boolean,
    onRequestPermissions: () -> Unit,
    onOpenSettings: () -> Unit
) {
    val missingLabels = missingPermissions.joinToString(", ") { it.toPrettyName() }
    val primaryMessage = "The app needs $missingLabels permission${if (missingPermissions.size > 1) "s" else ""} to scan and connect to your BLE hardware."
    val shouldOpenSettings = hasRequestedPermissions && !shouldShowPermissionRationale

    Column(
        modifier = Modifier.fillMaxSize(),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            primaryMessage,
            color = MaterialTheme.colorScheme.onBackground
        )
        Spacer(Modifier.height(12.dp))
        if (shouldShowPermissionRationale) {
            Text(
                "Please grant the requested permission${if (missingPermissions.size > 1) "s" else ""} to continue.",
                color = MaterialTheme.colorScheme.onBackground
            )
            Spacer(Modifier.height(12.dp))
        }
        Button(
            onClick = if (shouldOpenSettings) onOpenSettings else onRequestPermissions
        ) {
            Text(if (shouldOpenSettings) "Open Settings" else "Grant Permissions")
        }
        if (shouldOpenSettings) {
            Spacer(Modifier.height(8.dp))
            Text(
                "You denied the request earlier. Enable the permission${if (missingPermissions.size > 1) "s" else ""} in system settings.",
                color = MaterialTheme.colorScheme.onBackground
            )
        }
    }
}

private fun String.toPrettyName(): String = when (this) {
    Manifest.permission.ACCESS_FINE_LOCATION -> "Location"
    Manifest.permission.BLUETOOTH_SCAN -> "Nearby devices"
    Manifest.permission.BLUETOOTH_CONNECT -> "Bluetooth connect"
    else -> this.substringAfterLast('.')
}
