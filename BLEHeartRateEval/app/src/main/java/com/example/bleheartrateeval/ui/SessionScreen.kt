package com.example.bleheartrateeval.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Switch
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import com.example.bleheartrateeval.data.ble.BondStatus
import com.example.bleheartrateeval.data.ble.ConnectionState
import com.example.bleheartrateeval.data.ble.LinkStatus
import com.example.bleheartrateeval.data.model.Record
import com.example.bleheartrateeval.ui.components.PPGChart
import com.example.bleheartrateeval.ui.components.TelemetryChart
import no.nordicsemi.android.ble.callback.PhyCallback

@Composable
fun SessionScreen(
    linkStatus: LinkStatus,
    records: List<Record>,
    loggingEnabled: Boolean,
    onEnableLogging: (overwrite: Boolean) -> Unit,
    onDisableLogging: () -> Unit,
    lastRssi: Int?,
    canShareLog: Boolean,
    onShareLog: () -> Unit,
    onDisconnect: () -> Unit,
    onClearData: () -> Unit,
    rawLines: List<String> = emptyList(),
) {
    var showLoggingDialog by remember { mutableStateOf(false) }
    var selectedTab by rememberSaveable { mutableStateOf(0) }
    val tabs = listOf("Telemetry", "PPG Signals")

    Column(Modifier.fillMaxSize().padding(16.dp)) {
        StatusStrip(linkStatus = linkStatus)
        Spacer(Modifier.height(8.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = onDisconnect) { Text("Disconnect") }
                Button(onClick = onClearData) { Text("Clear") }
            }
            Row(
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text("Log to file", color = MaterialTheme.colorScheme.onBackground)
                Switch(
                    checked = loggingEnabled,
                    onCheckedChange = { checked ->
                        if (checked) {
                            showLoggingDialog = true
                        } else {
                            onDisableLogging()
                        }
                    }
                )
                TextButton(onClick = onShareLog, enabled = canShareLog) {
                    Text("Share")
                }
            }
        }
        if (showLoggingDialog) {
            AlertDialog(
                onDismissRequest = { showLoggingDialog = false },
                title = { Text("Enable file logging") },
                text = { Text("Choose whether to append to or overwrite today's log file.") },
                confirmButton = {
                    TextButton(onClick = {
                        onEnableLogging(false)
                        showLoggingDialog = false
                    }) { Text("Append") }
                },
                dismissButton = {
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        TextButton(onClick = {
                            onEnableLogging(true)
                            showLoggingDialog = false
                        }) { Text("Overwrite") }
                        TextButton(onClick = { showLoggingDialog = false }) { Text("Cancel") }
                    }
                }
            )
        }
        Spacer(Modifier.height(12.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            val rssiText = when {
                lastRssi == null -> "-"
                else -> "${lastRssi} dBm"
            }
            Text(
                "RSSI: $rssiText",
                color = MaterialTheme.colorScheme.onBackground
            )
            
            // Find the last record with a non-null HR value
            val lastHr = records.lastOrNull { it.hr != null }?.hr
            val hrText = lastHr?.toString() ?: "-"
            Text(
                "HR: $hrText bpm",
                color = MaterialTheme.colorScheme.onBackground
            )
        }
        
        // Tab navigation
        TabRow(selectedTabIndex = selectedTab) {
            tabs.forEachIndexed { index, title ->
                Tab(
                    selected = selectedTab == index,
                    onClick = { selectedTab = index },
                    text = { Text(title) }
                )
            }
        }
        
        Spacer(Modifier.height(8.dp))
        
        // Chart area fills available space
        when (selectedTab) {
            0 -> TelemetryChart(modifier = Modifier.weight(1f).fillMaxWidth(), data = records)
            1 -> PPGChart(modifier = Modifier.weight(1f).fillMaxWidth(), data = records)
        }
    }
}

@Composable
private fun StatusStrip(linkStatus: LinkStatus, modifier: Modifier = Modifier) {
    val surface = MaterialTheme.colorScheme.surfaceVariant
    val onSurface = MaterialTheme.colorScheme.onSurfaceVariant
    val warningColor = MaterialTheme.colorScheme.error
    val mtuValue = linkStatus.mtu
    val mtuWarning = mtuValue != null && mtuValue < 100
    val mtuColor = if (mtuWarning) warningColor else onSurface

    Row(
        modifier = modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(8.dp))
            .background(surface)
            .padding(horizontal = 12.dp, vertical = 10.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Column(modifier = Modifier.weight(1f, fill = true)) {
            Text(
                "Connection: ${connectionLabel(linkStatus.connectionState)}",
                color = onSurface,
                style = MaterialTheme.typography.bodyMedium
            )
            Text(
                "Bond: ${bondLabel(linkStatus.bondStatus)}",
                color = onSurface,
                style = MaterialTheme.typography.bodySmall
            )
        }
        Spacer(Modifier.width(12.dp))
        Column(horizontalAlignment = Alignment.End) {
            Text(
                "MTU: ${mtuValue?.toString() ?: "-"}",
                color = mtuColor,
                style = MaterialTheme.typography.bodyMedium
            )
            Text(
                "PHY: ${formatPhy(linkStatus.txPhy, linkStatus.rxPhy)}",
                color = onSurface,
                style = MaterialTheme.typography.bodySmall
            )
            if (mtuWarning) {
                Text(
                    "Low MTU",
                    color = warningColor,
                    style = MaterialTheme.typography.labelSmall
                )
            }
        }
    }
}

private fun connectionLabel(state: ConnectionState): String = when (state) {
    ConnectionState.Disconnected -> "Disconnected"
    ConnectionState.Connecting -> "Connecting"
    ConnectionState.Connected -> "Connected"
    ConnectionState.Ready -> "Ready"
    ConnectionState.Disconnecting -> "Disconnecting"
    is ConnectionState.Failed -> "Failed (${state.reason})"
}

private fun bondLabel(status: BondStatus): String = when (status) {
    BondStatus.Unknown -> "Unknown"
    BondStatus.NotBonded -> "Not bonded"
    BondStatus.Bonding -> "Bonding..."
    BondStatus.Bonded -> "Bonded"
}

private fun formatPhy(txPhy: Int?, rxPhy: Int?): String {
    val txName = phyName(txPhy)
    val rxName = phyName(rxPhy)
    return when {
        txPhy == null && rxPhy == null -> "-"
        txPhy == null -> rxName
        rxPhy == null -> txName
        txPhy == rxPhy -> txName
        else -> "$txName / $rxName"
    }
}

private fun phyName(value: Int?): String = when (value) {
    PhyCallback.PHY_LE_1M -> "LE 1M"
    PhyCallback.PHY_LE_2M -> "LE 2M"
    PhyCallback.PHY_LE_CODED -> "LE Coded"
    else -> "-"
}
