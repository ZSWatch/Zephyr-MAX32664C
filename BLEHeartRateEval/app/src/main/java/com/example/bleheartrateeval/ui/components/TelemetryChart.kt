package com.example.bleheartrateeval.ui.components

import android.content.Context
import android.graphics.Color
import android.graphics.DashPathEffect
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.viewinterop.AndroidView
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.ui.unit.dp
import com.example.bleheartrateeval.data.model.Record
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.AxisBase
import com.github.mikephil.charting.components.Legend
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.components.YAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import com.github.mikephil.charting.formatter.ValueFormatter
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.roundToInt

private val SKIN_CONTACT_LABELS = listOf(
    "Undetected",
    "Off skin",
    "On unknown",
    "On skin"
)

private val ACTIVITY_LABELS = listOf(
    "Rest",
    "Other",
    "Walk",
    "Run",
    "Bike"
)

@Composable
fun TelemetryChart(modifier: Modifier = Modifier, data: List<Record>) {
    Column(modifier = modifier) {
        // Top chart: HR (left axis) and Confidence (right axis)
        AndroidView(
            modifier = Modifier
                .fillMaxWidth()
                .weight(2f), // 2/3 of the space
            factory = { context: Context ->
                LineChart(context).apply { 
                    configureHRConfidenceChart()
                }
            },
            update = { chart ->
                updateHRConfidenceChart(chart, data)
            }
        )
        
        Spacer(modifier = Modifier.height(8.dp))
        
        // Bottom chart: Activity and Skin Contact
        AndroidView(
            modifier = Modifier
                .fillMaxWidth()
                .weight(1f), // 1/3 of the space
            factory = { context: Context ->
                LineChart(context).apply { 
                    configureActivityChart()
                }
            },
            update = { chart ->
                updateActivityChart(chart, data)
            }
        )
    }
}

// Update HR and Confidence chart
private fun updateHRConfidenceChart(chart: LineChart, data: List<Record>) {
    if (data.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    val hrEntries = mutableListOf<Entry>()
    val confEntries = mutableListOf<Entry>()
    val startTime = data.first().timestamp
    
    data.forEach { r ->
        val x = (r.timestamp - startTime) / 1000f
        r.hr?.let { hrEntries.add(Entry(x, it.toFloat())) }
        r.confidence?.let { confEntries.add(Entry(x, it)) }
    }

    if (hrEntries.isEmpty() && confEntries.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    val dataSets = mutableListOf<LineDataSet>()
    
    if (hrEntries.isNotEmpty()) {
        dataSets.add(LineDataSet(hrEntries, "HR").apply {
            axisDependency = YAxis.AxisDependency.LEFT
            color = Color.RED
            setDrawCircles(false)
            lineWidth = 2.5f
            setDrawValues(false)
            mode = LineDataSet.Mode.LINEAR
        })
    }
    
    if (confEntries.isNotEmpty()) {
        dataSets.add(LineDataSet(confEntries, "Confidence").apply {
            axisDependency = YAxis.AxisDependency.RIGHT
            color = Color.GREEN
            setDrawCircles(true)
            circleRadius = 2f
            setCircleColor(Color.GREEN)
            lineWidth = 1.5f
            setDrawValues(false)
            enableDashedLine(10f, 5f, 0f) // Dotted line
        })
    }

    chart.data = LineData(dataSets as List<com.github.mikephil.charting.interfaces.datasets.ILineDataSet>)

    // Set X-axis window (20 seconds)
    val allEntries = hrEntries + confEntries
    if (allEntries.isNotEmpty()) {
        val maxX = allEntries.maxOf { it.x }
        val minX = 0f
        chart.xAxis.axisMinimum = minX
        chart.xAxis.axisMaximum = maxX.coerceAtLeast(20f)
        chart.setVisibleXRangeMaximum(20f)
    }

    chart.notifyDataSetChanged()
    chart.invalidate()
}

// Update Activity chart
private fun updateActivityChart(chart: LineChart, data: List<Record>) {
    if (data.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    val activityEntries = mutableListOf<Entry>()
    val scEntries = mutableListOf<Entry>()
    val startTime = data.first().timestamp

    data.forEach { r ->
        val x = (r.timestamp - startTime) / 1000f
        r.activity?.let { activityEntries.add(Entry(x, it.toFloat())) }
        r.skinContact?.let { scEntries.add(Entry(x, it.toFloat())) }
    }

    if (activityEntries.isEmpty() && scEntries.isEmpty()) {
        chart.clear()
        chart.invalidate()
        return
    }

    val dataSets = mutableListOf<LineDataSet>()

    if (activityEntries.isNotEmpty()) {
        dataSets.add(LineDataSet(activityEntries, "Activity").apply {
            color = Color.CYAN
            setDrawCircles(false)
            lineWidth = 2.5f
            setDrawValues(false)
            mode = LineDataSet.Mode.LINEAR
            axisDependency = YAxis.AxisDependency.RIGHT
        })
    }

    if (scEntries.isNotEmpty()) {
        dataSets.add(LineDataSet(scEntries, "Skin Contact").apply {
            color = Color.YELLOW
            setDrawCircles(false)
            lineWidth = 2.5f
            setDrawValues(false)
            mode = LineDataSet.Mode.LINEAR
            axisDependency = YAxis.AxisDependency.LEFT
        })
    }

    chart.data = LineData(dataSets as List<com.github.mikephil.charting.interfaces.datasets.ILineDataSet>)

    // Set X-axis window (20 seconds)
    val allEntries = activityEntries + scEntries
    if (allEntries.isNotEmpty()) {
        val maxX = allEntries.maxOf { it.x }
        val minX = 0f
        chart.xAxis.axisMinimum = minX
        chart.xAxis.axisMaximum = maxX.coerceAtLeast(20f)
        chart.setVisibleXRangeMaximum(20f)
    }

    chart.notifyDataSetChanged()
    chart.invalidate()
}

// Configure HR and Confidence chart (with dual Y-axes)
private fun LineChart.configureHRConfidenceChart() {
    description.isEnabled = false
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
    xAxis.granularity = 30f
    xAxis.valueFormatter = object : ValueFormatter() {
        override fun getFormattedValue(value: Float): String {
            val mins = (value / 60).toInt()
            val secs = (value % 60).toInt()
            return String.format("%d:%02d", mins, secs)
        }
    }
    
    // Left Y-axis (HR)
    axisLeft.textColor = Color.WHITE
    axisLeft.textSize = 12f
    axisLeft.gridColor = Color.rgb(80, 80, 80)
    axisLeft.setDrawGridLines(true)
    axisLeft.axisLineColor = Color.WHITE
    axisLeft.axisLineWidth = 1f
    axisLeft.granularity = 1f  // Show labels every 1 bpm
    axisLeft.setLabelCount(10, false)
    axisLeft.setDrawLabels(true)
    
    // Right Y-axis (Confidence, 0-100%)
    axisRight.isEnabled = true
    axisRight.textColor = Color.WHITE
    axisRight.textSize = 12f
    axisRight.gridColor = Color.TRANSPARENT // Don't draw grid for right axis
    axisRight.setDrawGridLines(false)
    axisRight.axisLineColor = Color.WHITE
    axisRight.axisLineWidth = 1f
    axisRight.axisMinimum = 0f
    axisRight.axisMaximum = 100f
    axisRight.granularity = 20f
    
    // Legend
    legend.isEnabled = true
    legend.verticalAlignment = Legend.LegendVerticalAlignment.TOP
    legend.horizontalAlignment = Legend.LegendHorizontalAlignment.RIGHT
    legend.textColor = Color.WHITE
    legend.textSize = 12f
    legend.form = Legend.LegendForm.LINE
    legend.formLineWidth = 3f
    
    setExtraOffsets(12f, 12f, 12f, 12f)
}

// Configure Activity chart for activity and skin contact values
private fun LineChart.configureActivityChart() {
    description.isEnabled = false
    setTouchEnabled(true)
    isDragEnabled = true
    setScaleEnabled(true)
    setPinchZoom(true)
    axisRight.isEnabled = true
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
    xAxis.granularity = 30f
    xAxis.valueFormatter = object : ValueFormatter() {
        override fun getFormattedValue(value: Float): String {
            val mins = (value / 60).toInt()
            val secs = (value % 60).toInt()
            return String.format("%d:%02d", mins, secs)
        }
    }
    
    // Left Y-axis configuration (skin contact)
    axisLeft.textColor = Color.WHITE
    axisLeft.textSize = 12f
    axisLeft.gridColor = Color.rgb(80, 80, 80)
    axisLeft.setDrawGridLines(true)
    axisLeft.axisLineColor = Color.WHITE
    axisLeft.axisLineWidth = 1f
    axisLeft.axisMinimum = 0f
    axisLeft.axisMaximum = (SKIN_CONTACT_LABELS.size - 1).toFloat()
    axisLeft.granularity = 1f
    axisLeft.setLabelCount(SKIN_CONTACT_LABELS.size, true)
    axisLeft.valueFormatter = EnumValueFormatter(SKIN_CONTACT_LABELS)

    // Right Y-axis (activity)
    axisRight.textColor = Color.WHITE
    axisRight.textSize = 12f
    axisRight.gridColor = Color.TRANSPARENT
    axisRight.setDrawGridLines(false)
    axisRight.axisLineColor = Color.WHITE
    axisRight.axisLineWidth = 1f
    axisRight.axisMinimum = 0f
    axisRight.axisMaximum = (ACTIVITY_LABELS.size - 1).toFloat()
    axisRight.granularity = 1f
    axisRight.setLabelCount(ACTIVITY_LABELS.size, true)
    axisRight.valueFormatter = EnumValueFormatter(ACTIVITY_LABELS)
    
    // Legend
    legend.isEnabled = true
    legend.verticalAlignment = Legend.LegendVerticalAlignment.TOP
    legend.horizontalAlignment = Legend.LegendHorizontalAlignment.RIGHT
    legend.textColor = Color.WHITE
    legend.textSize = 12f
    legend.form = Legend.LegendForm.LINE
    legend.formLineWidth = 3f
    
    setExtraOffsets(12f, 12f, 12f, 12f)
}

private class EnumValueFormatter(
    private val labels: List<String>
) : ValueFormatter() {
    override fun getAxisLabel(value: Float, axis: AxisBase?): String {
        val index = value.roundToInt()
        return labels.getOrNull(index).orEmpty()
    }
}
