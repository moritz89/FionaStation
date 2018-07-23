
#ifndef ESP_MONITOR_MQTT
#define ESP_MONITOR_MQTT

#include <PubSubClient.h>

namespace eps_monitor {

class Mqtt {
 public:
  Mqtt(PubSubClient& client, const char* client_id)
      : client_(client), client_id_(client_id) {}

 private:
  PubSubClient& client_;
  const char* client_id;
}

void mqtt_connect(PubSubClient* client, const char* client_id) {
  /* Loop until reconnected */
  while (!client->connected()) {
    Serial.print("MQTT connecting ...");

    if (client->connect(client_id)) {
      Serial.println("connected");
      // subscribe topic with default QoS 0
      client->subscribe(LED_TOPIC);
    } else {
      Serial.print("failed, status code =");
      Serial.print(client->state());
      Serial.println("try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

}  // namespace eps_monitor

#endif