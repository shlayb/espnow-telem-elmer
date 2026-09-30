/*
Target: ESP32 klasik / ESP32-WROOM-32. Serial Monitor 115200 baud.
Kompatibel dengan PlatformIO (Arduino ESP32 Core 2.x & 3.x).
Upload Penerima dahulu, catat STA MAC, isi RECEIVER_MAC pada Pengirim.
32 byte, 8 kanal int16, FreeRTOS, target 200 Hz (SEND_PERIOD_MS=5).
PHY HT20 MCS7 SGI nominal 72.2 Mbps.
Daya TX dipetakan driver ke batas maksimum.
Data dummy: ganti fill_sensor_data. Skala 100 berarti nilai 1234 = 12.34.
Unicast tanpa enkripsi; MAC ACK bukan ACK aplikasi.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"

#include "esp_idf_version.h"
#if __has_include("esp_mac.h")
#include "esp_mac.h"
#endif

#ifndef MAC2STR
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#endif
#ifndef MACSTR
#define MACSTR "%02X:%02X:%02X:%02X:%02X:%02X"
#endif

// =========================================================================
// OPSI RISET TRANSMISI RADIO & FREKUENSI (Hapus '//' pada opsi yang ingin diuji)
// =========================================================================

// --- OPSI 1: FREKUENSI TRANSMISI (PERIODE SAMPEL) ---
#define SEND_PERIOD_MS 5   // [200 Hz] Ultra High Speed (Sangat cepat, cocok untuk telemetry balap realtime)
//#define SEND_PERIOD_MS 10  // [100 Hz] High Speed
//#define SEND_PERIOD_MS 20  // [50 Hz] Standard Telemetry
//#define SEND_PERIOD_MS 50  // [20 Hz] Low Refresh Rate

// --- OPSI 2: BITRATE PHY RADIO (RANGE VS SPEED) ---
#define RADIO_RATE WIFI_PHY_RATE_MCS7_SGI // [72.2 Mbps] Kecepatan Maksimum HT20 (Airtime paling singkat, Jarak dekat-menengah)
// #define RADIO_RATE WIFI_PHY_RATE_MCS0_LGI // [6.5 Mbps] Jangkauan jauh HT20 (Sensitivitas tinggi & kuat menembus rintangan)
//#define RADIO_RATE WIFI_PHY_RATE_11M_L    // [11 Mbps] Legacy 802.11b (Stabilitas tinggi di area banyak interference)
//#define RADIO_RATE WIFI_PHY_RATE_1M_L     // [1 Mbps] Legacy 802.11b Long Preamble (Sensitivitas Ekstrem / Jangkauan Maksimal)

// --- OPSI 3: CHANNEL WI-FI (2.4 GHz) ---
#define RADIO_CHANNEL 6   // Channel 6 (Default)
//#define RADIO_CHANNEL 1   // Channel 1
//#define RADIO_CHANNEL 11  // Channel 11

#define PACKET_MAGIC 0x4E45
#define PACKET_VERSION 1

// Urutan field menghindari padding: tidak perlu packed/akses tak selaras.
// Format biner little-endian, kedua ESP32 harus memakai definisi sama.
typedef struct {
    uint32_t session;       // random setiap boot
    uint32_t seq;           // nomor percobaan kirim
    uint32_t timestamp_us;  // micros pengirim, wrap ~71.6 menit
    int16_t channel[8];     // fixed-point; tentukan skala per kanal
    uint16_t magic;
    uint8_t version;
    uint8_t flags;          // 8 boolean dalam 1 byte
} packet_t;
static_assert(sizeof(packet_t) == 32, "Ukuran paket harus 32 byte");
static_assert(pdMS_TO_TICKS(SEND_PERIOD_MS) > 0, "Periode kurang dari 1 tick");

#define APP_LOG(tag, fmt, ...) Serial.printf("[%s] " fmt "\n", tag, ##__VA_ARGS__)
static void radio_init(void);
static void radio_init(void) {
    if (!WiFi.mode(WIFI_STA)) { Serial.println("WiFi init gagal"); abort(); }
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    int8_t limit;
    ESP_ERROR_CHECK(esp_wifi_get_max_tx_power(&limit));
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA,mac));
    APP_LOG("RADIO", "STA MAC=" MACSTR " channel=%d TX limit=%.2f dBm",
            MAC2STR(mac),RADIO_CHANNEL,limit/4.0);
    ESP_ERROR_CHECK(esp_now_init());
}

static void sent_cb(const uint8_t *, esp_now_send_status_t);
static void fill_sensor_data(packet_t *);
static void tx_task(void *);

// GANTI dengan STA MAC receiver yang muncul di monitor serial.
static const uint8_t RECEIVER_MAC[6] = {0x3C, 0x71, 0xBF, 0x7E, 0xC0, 0xC0};
static QueueHandle_t done_queue;

// Callback berjalan di task Wi-Fi, BUKAN ISR.
static void sent_cb(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    xQueueSend(done_queue, &status, 0);
}

static void fill_sensor_data(packet_t *p) {
    // SIMULASI SENSOR TELEMETRY GOKART (Fixed Point: Nilai x 100)
    static int16_t sim_speed = 4550; // 45.50 km/h
    sim_speed += random(-15, 20);
    if (sim_speed < 2000) sim_speed = 2000; // Min 20.00 km/h
    if (sim_speed > 8500) sim_speed = 8500; // Max 85.00 km/h

    int16_t sim_rpm = sim_speed * 85 / 100; // RPM proporsional ke kecepatan

    p->channel[0] = sim_speed;             // Speed (km/h x 100)
    p->channel[1] = sim_rpm;               // Motor RPM
    p->channel[2] = 4820 - (p->seq % 150); // Baterai (V x 100) e.g. 48.20 V
    p->channel[3] = 5540 + (random(0, 30));// Suhu Motor (°C x 100) e.g. 55.40 °C
    p->channel[4] = random(100, 500);      // Auxiliary / Sensor 4
    p->channel[5] = random(100, 500);      // Auxiliary / Sensor 5
    p->channel[6] = random(100, 500);      // Auxiliary / Sensor 6
    p->channel[7] = random(100, 500);      // Auxiliary / Sensor 7

    p->flags = 1;
}

static void tx_task(void *arg) {
    (void)arg;
    packet_t p = {};
    p.session=esp_random(); p.magic=PACKET_MAGIC; p.version=PACKET_VERSION;
    TickType_t deadline = xTaskGetTickCount();
    int64_t last_log = esp_timer_get_time();
    uint32_t ok=0, fail=0, api_error=0;
    for (;;) {
        fill_sensor_data(&p);
        p.timestamp_us = (uint32_t)esp_timer_get_time();
        esp_err_t e = esp_now_send(RECEIVER_MAC, (const uint8_t *)&p, sizeof(p));
        if (e == ESP_OK) {
            esp_now_send_status_t status;
            // Tepat satu paket in-flight. Timeout tidak boleh dilanjutkan
            // dengan kiriman baru: callback lama bisa tertukar dengan baru.
            if (xQueueReceive(done_queue, &status, pdMS_TO_TICKS(1000)) != pdTRUE) {
                APP_LOG("TX", "Callback timeout; restart untuk reset radio");
                esp_restart();
            }
            if (status == ESP_NOW_SEND_SUCCESS) ++ok; else ++fail;
        } else {
            ++api_error;
        }
        ++p.seq;
        int64_t now = esp_timer_get_time();
        if (now-last_log >= 1000000) {
            APP_LOG("TX", "MAC_OK=%" PRIu32 " MAC_FAIL=%" PRIu32 " API_ERR=%" PRIu32 ,
                     ok, fail, api_error);
            ok=fail=api_error=0;
            last_log=now;
        }
        // Jika terlambat, lewati jadwal lama, jangan kirim burst mengejar waktu.
        TickType_t ticks=xTaskGetTickCount();
        if ((TickType_t)(ticks-deadline) >= pdMS_TO_TICKS(SEND_PERIOD_MS)) deadline=ticks;
        vTaskDelayUntil(&deadline, pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}

void setup(void) {
    Serial.begin(115200);
    delay(500);
    radio_init();
    uint8_t zero[6]={0};
    if (!memcmp(RECEIVER_MAC, zero, 6) || (RECEIVER_MAC[0]&1)) {
        APP_LOG("TX", "Isi RECEIVER_MAC dengan MAC unicast receiver dahulu");
        return;
    }
    done_queue=xQueueCreate(1, sizeof(esp_now_send_status_t));
    assert(done_queue);
    ESP_ERROR_CHECK(esp_now_register_send_cb(sent_cb));
    esp_now_peer_info_t peer={};
    peer.channel=RADIO_CHANNEL; peer.ifidx=WIFI_IF_STA; peer.encrypt=false;
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

#if ESP_IDF_VERSION_MAJOR >= 5
    esp_now_rate_config_t rate={};
    rate.phymode=WIFI_PHY_MODE_HT20; rate.rate=RADIO_RATE;
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(RECEIVER_MAC, &rate));
#endif

    assert(xTaskCreate(tx_task,"tx",4096,NULL,5,NULL)==pdPASS);
}

void loop(void) { vTaskDelay(pdMS_TO_TICKS(1000)); }