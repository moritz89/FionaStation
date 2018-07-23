#ifndef ESP_MONITOR_CONFIGURATION
#define ESP_MONITOR_CONFIGURATION

#include "chrono"

namespace esp_monitor {

// WiFi
const char* ssid = "your-ssid";
const char* password = "your-password";

// MQTT
const char* mqtt_server = "192.168.0.1";
const char* client_id = "fiona_box1";
const std::chrono::duration<int> mqtt_reconnect_wait = std::chrono::seconds(5);
const char* lux_topic = "tele/hydro_box1/MAX44009_L";

}  // namespace esp_monitor

#endif
