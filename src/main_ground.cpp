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
#include <atomic>
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

#define RADIO_CHANNEL 6
#define SEND_PERIOD_MS 5 // target 200 paket/s; 20 = 50 Hz, 10 = 100 Hz
#define RADIO_RATE WIFI_PHY_RATE_MCS7_SGI // HT20: 72.2 Mbps nominal
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

static void rx_task(void *);
// Opsional: isi MAC sender. Nol = terima pengirim valid pertama setelah boot.
static const uint8_t SENDER_MAC[6]={0,0,0,0,0,0};
typedef struct { packet_t packet; uint8_t mac[6]; int8_t rssi; } rx_item_t;
static QueueHandle_t rx_queue;
static std::atomic<unsigned> queue_drop{0};

#if ESP_IDF_VERSION_MAJOR >= 5
static void received_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !data || len != static_cast<int>(sizeof(packet_t))) return;
    const uint8_t *src_addr = info->src_addr;
    int8_t rssi_val = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
#else
static void received_cb(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (!mac_addr || !data || len != static_cast<int>(sizeof(packet_t))) return;
    const uint8_t *src_addr = mac_addr;
    int8_t rssi_val = 0;
#endif
    uint8_t zero[6]={0};
    if (memcmp(SENDER_MAC,zero,6) && memcmp(SENDER_MAC,src_addr,6)) return;
    rx_item_t item;
    memcpy(&item.packet,data,sizeof(item.packet));
    if (item.packet.magic != PACKET_MAGIC || item.packet.version != PACKET_VERSION) return;
    memcpy(item.mac,src_addr,6);
    item.rssi=rssi_val;
    if (xQueueSend(rx_queue,&item,0)!=pdTRUE) queue_drop.fetch_add(1, std::memory_order_relaxed);
}

static void rx_task(void *arg) {
    (void)arg;
    bool locked=false, have_session=false;
    uint8_t source[6]={0};
    uint32_t session=0, last_seq=0, received=0, gap=0, duplicate_old=0;
    int32_t rssi_sum=0;
    uint32_t rssi_count=0;
    int8_t last_rssi=0;
    packet_t latest={0};
    int64_t last_log=esp_timer_get_time(), last_rx=0;
    for (;;) {
        rx_item_t item;
        if (xQueueReceive(rx_queue,&item,pdMS_TO_TICKS(100))==pdTRUE) {
            if (!locked) { memcpy(source,item.mac,6); locked=true; }
            if (!memcmp(source,item.mac,6)) {
                packet_t *p=&item.packet;
                bool accept=true;
                if (have_session && session==p->session) {
                    uint32_t delta=p->seq-last_seq; // menangani wrap uint32
                    if (delta==0 || delta>=0x80000000u) { ++duplicate_old; accept=false; }
                    else gap+=delta-1;
                }
                if (accept) {
                    have_session=true; session=p->session; last_seq=p->seq;
                    latest=*p;
                    last_rssi=item.rssi;
                    rssi_sum+=item.rssi;
                    ++rssi_count;
                    ++received;
                    last_rx=esp_timer_get_time();
                    // Proses seluruh kanal di sini. Jangan print setiap paket.
                    // Jika diteruskan ke task lain, gunakan queue/mutex.
                }
            }
        }
        int64_t now=esp_timer_get_time();
        int64_t elapsed_us=now-last_log;
        if (elapsed_us>=1000000) {
            float dt = elapsed_us / 1000000.0f;
            uint32_t total_expected = received + gap;
            float packet_loss_pct = (total_expected > 0) ? ((float)gap / (float)total_expected * 100.0f) : 0.0f;
            float throughput_bps = (float)(received * sizeof(packet_t)) / dt;
            float throughput_kbps = (throughput_bps * 8.0f) / 1000.0f;
            int avg_rssi = (rssi_count > 0) ? (int)(rssi_sum / (int32_t)rssi_count) : last_rssi;

            float speed_kmh = latest.channel[0] / 100.0f;
            int16_t rpm = latest.channel[1];
            float batt_v = latest.channel[2] / 100.0f;
            float temp_c = latest.channel[3] / 100.0f;

            APP_LOG("RX", "rx=%" PRIu32 " gap=%" PRIu32 " loss=%.1f%% rssi=%d dBm tput=%.2f B/s (%.1f kbps) qdrop=%u | Speed=%.2fkm/h RPM=%d Batt=%.2fV Temp=%.2f°C",
                     received, gap, packet_loss_pct, avg_rssi, throughput_bps, throughput_kbps,
                     queue_drop.exchange(0, std::memory_order_relaxed),
                     speed_kmh, rpm, batt_v, temp_c);
            received=gap=duplicate_old=rssi_count=rssi_sum=0; last_log=now;
        }
    }
}

void setup(void) {
    Serial.begin(115200);
    delay(500);
    rx_queue=xQueueCreate(32,sizeof(rx_item_t));
    assert(rx_queue);
    radio_init();
    ESP_ERROR_CHECK(esp_now_register_recv_cb(received_cb));
    assert(xTaskCreate(rx_task,"rx",4096,NULL,5,NULL)==pdPASS);
}

void loop(void) { vTaskDelay(pdMS_TO_TICKS(1000)); }