# Fiona Box

## Before Flashing

Before compiling and flashing your ESP32, rename the `secrets.h.sample` file to `secrets.h` in the `src` folder. The following variables have to be set

- password
- ssid
- mqtt_server

As optional variables, `client_id` is used to scope the MQTT messages.