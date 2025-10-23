package com.example.bleheartrateeval.data.ble

import android.content.Context

/**
 * Provides a shared [BleRepo] instance so the UI and background services
 * operate on the same connection state.
 */
object BleRepoHolder {
    @Volatile
    private var instance: BleRepo? = null

    fun get(context: Context): BleRepo {
        val appContext = context.applicationContext
        return instance ?: synchronized(this) {
            instance ?: BleRepo(appContext).also { instance = it }
        }
    }
}
