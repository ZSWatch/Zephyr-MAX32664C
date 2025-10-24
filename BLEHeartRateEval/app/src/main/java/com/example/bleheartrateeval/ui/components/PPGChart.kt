package com.example.bleheartrateeval.ui.components

import android.content.Context
import android.graphics.Color
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import com.example.bleheartrateeval.data.model.Record
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.Legend
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.github.mikephil.charting.formatter.ValueFormatter

private const val PPG_WINDOW_MS = 20_000L
private const val PPG_VISIBLE_SECONDS = 20f
private const val PPG_UPDATE_INTERVAL_MS = 100L
private const val PPG_MIN_SPAN = 100f
private const val PPG_PADDING_SCALE = 1.1f

@Composable
fun PPGChart(modifier: Modifier = Modifier, data: List<Record>) {
    Column(modifier = modifier) {
        // GREEN1 Chart
        AndroidView(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
            factory = { context: Context ->
                LineChart(context).apply {
                    configurePPGChart("GREEN1")
                }
            },
            update = { chart ->
                updateSinglePPGChart(chart, data, "GREEN1", Color.rgb(0, 255, 0))
            }
        )

        Spacer(modifier = Modifier.height(8.dp))

        // GREEN2 Chart
        AndroidView(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f),
            factory = { context: Context ->
                LineChart(context).apply {
                    configurePPGChart("GREEN2")
                }
            },
            update = { chart ->
                updateSinglePPGChart(chart, data, "GREEN2", Color.rgb(100, 200, 100))
            }
        )
    }
}

private fun updateSinglePPGChart(chart: LineChart, data: List<Record>, signalName: String, color: Int) {
    if (data.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    if (chart.shouldThrottle(PPG_UPDATE_INTERVAL_MS)) {
        return
    }

    val windowData = data.windowRecent(PPG_WINDOW_MS)

    if (windowData.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    val entries = ArrayList<Entry>(windowData.size)
    val startTime = windowData.first().timestamp

    windowData.forEach { r ->
        val x = (r.timestamp - startTime) / 1000f
        
        // Extract signal from extras (remove unit if present)
        r.extras[signalName]?.substringBefore(' ')?.toFloatOrNull()?.let {
            entries.add(Entry(x, it))
        }
    }

    if (entries.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    val dataSet = LineDataSet(entries, signalName).apply {
        this.color = color
        setDrawCircles(false)
        lineWidth = 1.5f
        setDrawValues(false)
        mode = LineDataSet.Mode.LINEAR
    }

    chart.data = LineData(dataSet)

    // Auto-scale and set X-axis window (20 seconds)
    val maxX = entries.maxOf { it.x }
    chart.xAxis.axisMinimum = 0f
    chart.xAxis.axisMaximum = maxX.coerceAtLeast(PPG_VISIBLE_SECONDS)
    chart.setVisibleXRangeMaximum(PPG_VISIBLE_SECONDS)

    val minY = entries.minOf { it.y }
    val maxY = entries.maxOf { it.y }
    val span = (maxY - minY).coerceAtLeast(PPG_MIN_SPAN)
    val halfSpan = span / 2f * PPG_PADDING_SCALE
    val mid = (maxY + minY) / 2f
    chart.axisLeft.axisMinimum = mid - halfSpan
    chart.axisLeft.axisMaximum = mid + halfSpan

    chart.notifyDataSetChanged()
    chart.invalidate()
}

private fun LineChart.configurePPGChart(title: String) {
    description.text = title
    description.textColor = Color.WHITE
    description.textSize = 14f
    description.isEnabled = true
    setTouchEnabled(true)
    isDragEnabled = true
    setScaleEnabled(true)
    setPinchZoom(true)
    setNoDataTextColor(Color.WHITE)
    setBackgroundColor(Color.rgb(30, 30, 30))
    
    // X-axis configuration
    xAxis.position = XAxis.XAxisPosition.BOTTOM
    xAxis.textColor = Color.WHITE
    xAxis.textSize = 12f
    xAxis.gridColor = Color.rgb(80, 80, 80)
    xAxis.setDrawGridLines(true)
    xAxis.axisLineColor = Color.WHITE
    xAxis.axisLineWidth = 1f
    xAxis.granularity = 5f
    xAxis.valueFormatter = object : ValueFormatter() {
        override fun getFormattedValue(value: Float): String {
            return String.format("%.1fs", value)
        }
    }
    
    // Left Y-axis (auto-scale)
    axisLeft.textColor = Color.WHITE
    axisLeft.textSize = 12f
    axisLeft.gridColor = Color.rgb(80, 80, 80)
    axisLeft.setDrawGridLines(true)
    axisLeft.axisLineColor = Color.WHITE
    axisLeft.axisLineWidth = 1f
    axisLeft.setDrawLabels(true)
    
    // Disable right Y-axis
    axisRight.isEnabled = false
    
    // Legend
    legend.isEnabled = false
    
    setExtraOffsets(12f, 12f, 12f, 12f)
}
