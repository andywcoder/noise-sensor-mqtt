# noise-sensor-mqtt

A noise sensor based on the ESP32 SoC that publishes its state over MQTT.

## Getting started

1. Download and install <a href="https://platformio.org/platformio-ide">PlatformIO IDE for Visual Studio Code or Atom</a>
2. Clone or download this respository
3. Open the solution folder in PlatformIO
4. Create the file src/secrets.h with the following preprocessing directives
	* #define SECRECT_WIFI_SSID "ssid"
    * #define SECRECT_WIFI_PASSWORD "password"
	* #define SECRECT_MQTT_HOST IPAddress(192, 168, 0, 1)
	* #define SECRECT_MQTT_PORT 1883
	* #define SECRECT_MQTT_USERNAME "username"
	* #define SECRECT_MQTT_PASSWORD "password"
5. Build the binary
