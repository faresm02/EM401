#include <WiFi.h>
#include <esp_now.h>

uint8_t flightMAC[] = {0x1C, 0xDB, 0xD4, 0x86, 0x35, 0x64};

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send FAIL");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_send_cb(onSent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, flightMAC, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
}

void loop() {
    const char *msg = "ping";
    esp_now_send(flightMAC, (uint8_t *)msg, strlen(msg));
    Serial.println("Sent ping");
    delay(1000);
}