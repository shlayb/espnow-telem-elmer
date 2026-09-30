#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include <Arduino.h>

// Struct data telemetry yang dikirim dari Air (Gokart) ke Ground Station via ESP-NOW
typedef struct __attribute__((packed)) {
    uint32_t messageId;       // ID Pesan / Counter
    uint32_t timestampMs;     // Waktu sistem (millis)
    float speedKmh;          // Kecepatan (km/h)
    uint16_t rpm;             // Putaran Mesin / Motor (RPM)
    float batteryVoltage;    // Tegangan Baterai (V)
    float tempCelsius;       // Temperatur Motor/ESC (°C)
    uint8_t statusFlags;      // Flag status (0 = OK, bitmask untuk error)
} TelemetryData;

#endif // TELEMETRY_DATA_H
