#ifndef ESP_MONITOR_CONFIGURATION
#define ESP_MONITOR_CONFIGURATION

#include "chrono"

namespace esp_monitor {

// WiFi
const char* ssid = "your-ssid";
const std::chrono::seconds wifi_connect_timeout(20);

// MQTT
const char* mqtt_server = "192.168.0.1";
const char* client_id = "fiona_box1";
const uint connection_attempts = 3;  // Maximum attempts before aborting
const char* lux_topic = "tele/hydro_box1/MAX44009_L";

// Pins
const uint status_led = 2;  // Pin to the status on-board LED

}  // namespace esp_monitor

#endif
