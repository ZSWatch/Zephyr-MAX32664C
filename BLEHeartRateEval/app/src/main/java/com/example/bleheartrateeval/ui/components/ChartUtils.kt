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

internal fun List<Record>.windowStartIndex(windowMs: Long): Int {
    if (isEmpty()) return -1
    val latestTimestamp = last().timestamp
    var index = size - 1
    while (index >= 0 && latestTimestamp - this[index].timestamp <= windowMs) {
        index--
    }
    val candidate = index + 1
    return if (candidate < size) candidate else size - 1
}
