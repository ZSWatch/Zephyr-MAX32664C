package com.example.bleheartrateeval.data

import android.content.Context
import java.io.File
import java.nio.charset.StandardCharsets
import java.time.Instant
import java.time.ZoneOffset
import java.time.format.DateTimeFormatter
import java.time.LocalDate

class LogWriter(private val context: Context) {
    private val dateFormatter: DateTimeFormatter = DateTimeFormatter.ISO_LOCAL_DATE
    private val timestampFormatter: DateTimeFormatter =
        DateTimeFormatter.ISO_INSTANT.withZone(ZoneOffset.UTC)

    private fun logDir(): File = File(context.filesDir, "logs").apply { mkdirs() }

    private fun fileFor(date: LocalDate): File =
        File(logDir(), "${date.format(dateFormatter)}.txt")

    fun todayFile(): File = fileFor(LocalDate.now(ZoneOffset.UTC))

    fun clearToday() {
        val file = todayFile()
        if (file.exists()) file.delete()
    }

    fun appendRaw(timestampMillis: Long, rawPayload: String) {
        val ts = timestampFormatter.format(Instant.ofEpochMilli(timestampMillis))
        val line = "$ts $rawPayload\n"
        val file = todayFile()
        file.appendText(line, StandardCharsets.UTF_8)
    }
}
