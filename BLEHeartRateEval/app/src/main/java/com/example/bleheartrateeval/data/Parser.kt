package com.example.bleheartrateeval.data

import com.example.bleheartrateeval.data.model.Record
import java.util.Locale

object Parser {
    fun parse(raw: String, timestamp: Long): Record {
        var hr: Int? = null
        var activity: Int? = null
        var skinContact: Int? = null
        var confidence: Float? = null
        val extras = mutableMapOf<String, String>()

        raw.split(';').forEach { section ->
            val trimmed = section.trim()
            if (trimmed.isEmpty()) return@forEach

            val parts = trimmed.split(',', limit = 3)
            if (parts.size < 2) return@forEach

            val keyOriginal = parts[0].trim()
            if (keyOriginal.isEmpty()) return@forEach
            val key = keyOriginal.lowercase(Locale.US)

            val value = parts.getOrNull(1)?.trim().orEmpty()
            val unit = parts.getOrNull(2)?.trim().orEmpty()

            when (key) {
                "hr" -> hr = value.toIntOrNull()
                "hr_conf", "hrconf", "hrconfidence", "confidence" -> confidence = value.toFloatOrNull()
                "activity", "act" -> activity = value.toIntOrNull()
                "sc", "skincontact" -> skinContact = value.toIntOrNull()
                else -> {
                    if (value.isNotEmpty() || unit.isNotEmpty()) {
                        extras[keyOriginal] = buildValueWithUnit(value, unit)
                    }
                }
            }
        }

        return Record(
            timestamp = timestamp,
            hr = hr,
            activity = activity,
            skinContact = skinContact,
            confidence = confidence,
            extras = extras,
            raw = raw
        )
    }

    private fun buildValueWithUnit(value: String, unit: String): String {
        return if (unit.isNotEmpty()) "$value $unit" else value
    }
}
