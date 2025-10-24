package com.example.bleheartrateeval.ui.components

import android.os.SystemClock
import com.example.bleheartrateeval.data.model.Record
import com.github.mikephil.charting.charts.LineChart

private class ChartUpdateState(var lastUpdateAt: Long = 0L)

internal fun LineChart.shouldThrottle(minIntervalMs: Long): Boolean {
    val state = (tag as? ChartUpdateState) ?: ChartUpdateState().also { tag = it }
    val now = SystemClock.elapsedRealtime()
    val skipUpdate = data != null && now - state.lastUpdateAt < minIntervalMs
    if (!skipUpdate) {
        state.lastUpdateAt = now
    }
    return skipUpdate
}

internal fun List<Record>.windowRecent(windowMs: Long): List<Record> {
    if (isEmpty()) return emptyList()
    val latestTimestamp = last().timestamp
    val window = ArrayList<Record>()
    for (i in indices.reversed()) {
        val record = this[i]
        if (latestTimestamp - record.timestamp > windowMs) {
            break
        }
        window.add(record)
    }
    window.reverse()
    return window
}
