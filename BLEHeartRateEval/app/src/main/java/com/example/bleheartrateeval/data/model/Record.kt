package com.example.bleheartrateeval.data.model

data class Record(
    val timestamp: Long,
    val hr: Int?,
    val activity: Int?,
    val skinContact: Int?,
    val confidence: Float?,
    val extras: Map<String, String> = emptyMap(),
    val raw: String
)
