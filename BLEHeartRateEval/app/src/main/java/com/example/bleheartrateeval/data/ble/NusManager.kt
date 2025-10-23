package com.example.bleheartrateeval.data.ble

import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothDevice
import android.content.Context
import android.os.Build
import android.util.Log
import no.nordicsemi.android.ble.BleManager
import no.nordicsemi.android.ble.PhyRequest
import no.nordicsemi.android.ble.callback.DataReceivedCallback
import no.nordicsemi.android.ble.callback.MtuCallback
import no.nordicsemi.android.ble.callback.PhyCallback
import java.util.UUID

/**
 * Minimal Nordic UART Service manager using Nordic BLE Library.
 * Enables notifications on TX and exposes received data via [onDataReceived].
 */
class NusManager(context: Context) : BleManager(context) {

    private var nusTx: BluetoothGattCharacteristic? = null // Notify from peripheral
    private var nusRx: BluetoothGattCharacteristic? = null // Write to peripheral

    var onDataReceived: ((ByteArray) -> Unit)? = null
    var onMtuChanged: ((Int) -> Unit)? = null
    var onPhyChanged: ((Int, Int) -> Unit)? = null

    override fun getGattCallback(): BleManagerGattCallback = object : BleManagerGattCallback() {
        override fun isRequiredServiceSupported(gatt: BluetoothGatt): Boolean {
            val service: BluetoothGattService? =
                gatt.getService(NUS_SERVICE_UUID)
            if (service != null) {
                nusRx = service.getCharacteristic(NUS_RX_CHAR_UUID)
                nusTx = service.getCharacteristic(NUS_TX_CHAR_UUID)
            }
            Log.d(TAG, "services: NUS service=${service!=null} tx=${nusTx!=null} rx=${nusRx!=null}")
            // Require at least TX for notifications.
            return nusTx != null
        }

        override fun initialize() {
            val device: BluetoothDevice? = bluetoothDevice
            if (device != null && device.bondState != BluetoothDevice.BOND_BONDED) {
                Log.d(TAG, "initialize: ensuring bond with ${device.address}")
                ensureBond()
                    .done {
                        Log.d(TAG, "ensureBond: success")
                    }
                    .fail { _, status ->
                        Log.w(TAG, "ensureBond: failed status=$status")
                    }
                    .enqueue()
            } else {
                Log.d(TAG, "initialize: device already bonded")
            }

            Log.d(TAG, "initialize: requesting MTU 247")
            requestMtu(247)
                .with(MtuCallback { _, mtu ->
                    Log.d(TAG, "MTU updated: $mtu")
                    onMtuChanged?.invoke(mtu)
                })
                .fail { _, status ->
                    Log.w(TAG, "requestMtu failed: status=$status current=${getMtu()}")
                    onMtuChanged?.invoke(getMtu())
                }
                .enqueue()

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                Log.d(TAG, "initialize: requesting LE 2M PHY")
                setPreferredPhy(
                    PhyRequest.PHY_LE_2M_MASK or PhyRequest.PHY_LE_1M_MASK,
                    PhyRequest.PHY_LE_2M_MASK or PhyRequest.PHY_LE_1M_MASK,
                    PhyRequest.PHY_OPTION_NO_PREFERRED
                )
                    .with(PhyCallback { _, txPhy, rxPhy ->
                        Log.d(TAG, "preferred PHY applied: tx=$txPhy rx=$rxPhy")
                        onPhyChanged?.invoke(txPhy, rxPhy)
                    })
                    .fail { _, status ->
                        Log.w(TAG, "setPreferredPhy failed: status=$status")
                    }
                    .enqueue()

                readPhy()
                    .with(PhyCallback { _, txPhy, rxPhy ->
                        Log.d(TAG, "readPhy: tx=$txPhy rx=$rxPhy")
                        onPhyChanged?.invoke(txPhy, rxPhy)
                    })
                    .fail { _, status ->
                        Log.w(TAG, "readPhy failed: status=$status")
                    }
                    .enqueue()
            } else {
                onPhyChanged?.invoke(PhyCallback.PHY_LE_1M, PhyCallback.PHY_LE_1M)
            }

            setNotificationCallback(nusTx)
                .with(DataReceivedCallback { _, data ->
                    val value = data.value
                    if (value != null) {
                        onDataReceived?.invoke(value)
                    }
                })
            Log.d(TAG, "initialize: enabling notifications on TX")
            enableNotifications(nusTx).enqueue()
        }

        override fun onServicesInvalidated() {
            Log.d(TAG, "onServicesInvalidated")
            nusRx = null
            nusTx = null
        }
    }

    fun writeRx(data: ByteArray) {
        val c = nusRx ?: return
        Log.d(TAG, "writeRx: len=${data.size}")
        writeCharacteristic(c, data, BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
            .enqueue()
    }

    fun readRssiOnce(onResult: (Int) -> Unit) {
        readRssi()
            .with { _, rssi -> onResult(rssi) }
            .fail { _, _ -> }
            .enqueue()
    }

    /**
     * Expose queue cancellation outside the manager so the repository can
     * abort pending operations before disconnecting.
     */
    fun cancelPendingOperations() {
        Log.d(TAG, "cancelPendingOperations")
        cancelQueue()
    }

    companion object {
        private const val TAG = "NusManager"
        val NUS_SERVICE_UUID: UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
        val NUS_RX_CHAR_UUID: UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")
        val NUS_TX_CHAR_UUID: UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
    }
}
