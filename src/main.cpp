#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include "secrets.h"

#define max(a, b) ((a) > (b) ? (a) : (b))

#define WIFI_SSID SECRECT_WIFI_SSID
#define WIFI_PASSWORD SECRECT_WIFI_PASSWORD

#define MQTT_HOST SECRECT_MQTT_HOST
#define MQTT_PORT SECRECT_MQTT_PORT
#define MQTT_USERNAME SECRECT_MQTT_USERNAME
#define MQTT_PASSWORD SECRECT_MQTT_PASSWORD

#define MQTT_TOPIC_STATE "home/ella_room/sensor/state"
#define MQTT_TOPIC_STATE_READY "READY"
#define MQTT_TOPIC_STATE_ENABLED "ENABLED"
#define MQTT_TOPIC_STATE_DISABLED "DISABLED"
#define MQTT_TOPIC_COMMAND "home/ella_room/sensor/command"
#define MQTT_TOPIC_COMMAND_ENABLE "ENABLE"
#define MQTT_TOPIC_COMMAND_DISABLE "DISABLE"
#define MQTT_TOPIC_COMMAND_CHECK "CHECK"
#define MQTT_TOPIC_NOISE "home/ella_room/sensor/noise"
#define MQTT_TOPIC_NOISE_ON "ON"
#define MQTT_TOPIC_NOISE_OFF "OFF"

#define ANALOG_PIN_SOUND 36
#define DIGITAL_PIN_SOUND 16
#define DIGITAL_PIN_LED 2

#define READ_DIGITAL_SENSOR false

#define SAMPLE_BUFFER_SIZE 16
uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
uint8_t sampleBufferIndex = 0;

#define NOISE_BUFFER_SIZE 4
unsigned long noiseBuffer[NOISE_BUFFER_SIZE];
uint8_t noiseBufferIndex = 0;

AsyncMqttClient mqttClient;
TimerHandle_t noiseDetectedTimer;
TimerHandle_t hasNoiseBufferReachedLimitTimer;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

bool isEnabled = false;

void publishMessage(char *topic, char *payload)
{
  Serial.print("MQTT publish ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(payload);
  mqttClient.publish(topic, 1, true, payload);
}

void handleMessage(char *topic, char *payload)
{
  if (strcmp(topic, MQTT_TOPIC_COMMAND) == 0)
  {
    Serial.print("Handle command ");
    Serial.println(payload);
	  
    if (strcmp(payload, MQTT_TOPIC_COMMAND_ENABLE) == 0)
    {
      Serial.println("Enable sensor");

      isEnabled = true;

      publishMessage(MQTT_TOPIC_STATE, MQTT_TOPIC_STATE_ENABLED);
    }
    else if (strcmp(payload, MQTT_TOPIC_COMMAND_DISABLE) == 0)
    {
      Serial.println("Disable sensor");

      isEnabled = false;

      publishMessage(MQTT_TOPIC_STATE, MQTT_TOPIC_STATE_DISABLED);
    }
    else if (strcmp(payload, MQTT_TOPIC_COMMAND_CHECK) == 0)
    {
      if(isEnabled)
      {
        Serial.println("Sensor is enabled");

        publishMessage(MQTT_TOPIC_STATE, MQTT_TOPIC_STATE_ENABLED);
      }
      else
      {
        Serial.println("Sensor is disabled");

        publishMessage(MQTT_TOPIC_STATE, MQTT_TOPIC_STATE_DISABLED);
      }
    }
  }
}

void noiseDetectedTimerCallback()
{
  Serial.println("Sensor LED off");
  digitalWrite(DIGITAL_PIN_LED, LOW);
}

void hasNoiseBufferReachedLimitTimerCallback()
{
  Serial.println("Sensor LED off");
  digitalWrite(DIGITAL_PIN_LED, LOW);

  publishMessage(MQTT_TOPIC_NOISE, MQTT_TOPIC_NOISE_OFF);
}

void updateSampleBuffer(uint16_t sample)
{
  sampleBuffer[sampleBufferIndex] = sample;
  sampleBufferIndex++;
  if (sampleBufferIndex >= SAMPLE_BUFFER_SIZE)
  {
    sampleBufferIndex = 0;
  }
}

void clearSampleBuffer()
{
  for (uint8_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
  {
    sampleBuffer[i] = 0;
  }
}

uint16_t getSample(int16_t index)
{
  if (index < 0)
  {
    index = SAMPLE_BUFFER_SIZE + index;
  }
  return sampleBuffer[index];
}

bool haveSampleBufferSamplesReachedLimit(uint16_t limit, uint8_t numberOfSamples)
{
  int16_t lastIndex = INT16_MIN;
  uint16_t number = 0;
  uint16_t maxNumber = 0;
  for (int16_t i = -numberOfSamples + 1; i < SAMPLE_BUFFER_SIZE; i++)
  {
    if (getSample(i) >= limit && (lastIndex == INT16_MIN || lastIndex == (i - 1)))
    {
      lastIndex = i;
      number++;
      maxNumber = max(number, maxNumber);
      if (maxNumber >= numberOfSamples)
      {
        return true;
      }
    }
    else
    {
      lastIndex = INT16_MIN;
      number = 0;
    }
  }
  return false;
}

void updateNoiseBuffer()
{
  noiseBuffer[noiseBufferIndex] = millis();
  noiseBufferIndex++;
  if (noiseBufferIndex >= NOISE_BUFFER_SIZE)
  {
    noiseBufferIndex = 0;
  }

  for (uint8_t i = 0; i < NOISE_BUFFER_SIZE; i++)
  {
    if (millis() - noiseBuffer[i] > 60000)
    {
      noiseBuffer[i] = 0;
    }
  }
}

void clearNoiseBuffer()
{
  for (uint8_t i = 0; i < NOISE_BUFFER_SIZE; i++)
  {
    noiseBuffer[i] = 0;
  }
}

bool hasNoiseBufferReachedLimit(uint8_t limit)
{
  uint8_t numberOfNoises = 0;
  for (uint8_t i = 0; i < NOISE_BUFFER_SIZE; i++)
  {
    if (noiseBuffer[i] != 0)
    {
      numberOfNoises++;
      if (numberOfNoises >= limit)
      {
        return true;
      }
    }
  }
  return false;
}

void connectToWifi()
{
  Serial.println("Wifi connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("MQTT connecting...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("Wifi event %d", event);
  Serial.println();
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("Wifi connected");
    Serial.print("  IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("Wifi connection lost");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("MQTT connected");
  Serial.print("  Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetId = mqttClient.subscribe(MQTT_TOPIC_COMMAND, 1);
  Serial.print("MQTT subscribing ");
  Serial.println(MQTT_TOPIC_COMMAND);
  Serial.print("  PacketId: ");
  Serial.println(packetId);

  publishMessage(MQTT_TOPIC_STATE, MQTT_TOPIC_STATE_READY);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("MQTT disconnected");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("MQTT subscription acknowledged");
  Serial.print("  PacketId: ");
  Serial.println(packetId);
  Serial.print("  Qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("MQTT unsubscription acknowledged");
  Serial.print("  PacketId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("MQTT message received");
  Serial.print("  Topic: ");
  Serial.println(topic);
  Serial.print("  Payload: ");
  Serial.println(payload);
  Serial.print("  Qos: ");
  Serial.println(properties.qos);
  Serial.print("  Dup: ");
  Serial.println(properties.dup);
  Serial.print("  Retain: ");
  Serial.println(properties.retain);
  Serial.print("  Len: ");
  Serial.println(len);
  Serial.print("  Index: ");
  Serial.println(index);
  Serial.print("  Total: ");
  Serial.println(total);

  char payload_length_fixed[len + 1];
  memcpy(payload_length_fixed, &payload[0], len);
  payload_length_fixed[len] = '\0';

  handleMessage(topic, payload_length_fixed);
}

void onMqttPublish(uint16_t packetId)
{
  Serial.println("MQTT publish acknowledged");
  Serial.print("  PacketId: ");
  Serial.println(packetId);
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  noiseDetectedTimer = xTimerCreate("noiseDetectedTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(noiseDetectedTimerCallback));
  hasNoiseBufferReachedLimitTimer = xTimerCreate("hasNoiseBufferReachedLimitTimer", pdMS_TO_TICKS(4000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(hasNoiseBufferReachedLimitTimerCallback));
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

  connectToWifi();

  pinMode(ANALOG_PIN_SOUND, INPUT);
  pinMode(DIGITAL_PIN_SOUND, INPUT);
  pinMode(DIGITAL_PIN_LED, OUTPUT);

  digitalWrite(DIGITAL_PIN_LED, LOW);
}

void loop()
{
  if (isEnabled)
  {
    uint16_t digitalSample = digitalRead(DIGITAL_PIN_SOUND);
    uint16_t analogSample = analogRead(ANALOG_PIN_SOUND);
    delay(10);
    analogSample += analogRead(ANALOG_PIN_SOUND);
    delay(10);
    analogSample += analogRead(ANALOG_PIN_SOUND);
    delay(10);
    analogSample += analogRead(ANALOG_PIN_SOUND);
    analogSample /= 4;
    uint16_t sample = READ_DIGITAL_SENSOR ? digitalSample == 0 ? 4095 : 0 : 4095 - analogSample;
    updateSampleBuffer(sample);

    float analogSampleVoltage = (float)analogSample / 4095.0f * 3.3f;
    String debugInfo;
    debugInfo += "Sensor";
    debugInfo += "\r\n  digitalSample: ";
    debugInfo += String(digitalSample);
    debugInfo += "\r\n  analogSample: ";
    debugInfo += String(analogSample);
    debugInfo += "\r\n  analogSampleVoltage: ";
    debugInfo += String(analogSampleVoltage, 3);
    debugInfo += "\r\n  sampleBuffer: ";
    for (uint8_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
    {
      debugInfo += String(i);
      debugInfo += "=";
      debugInfo += String(sampleBuffer[i]);
      if (i < SAMPLE_BUFFER_SIZE - 1)
      {
        debugInfo += ", ";
      }
    }
    debugInfo += "\r\n  noiseBuffer: ";
    for (uint8_t i = 0; i < NOISE_BUFFER_SIZE; i++)
    {
      debugInfo += String(i);
      debugInfo += "=";
      debugInfo += String(noiseBuffer[i]);
      if (i < NOISE_BUFFER_SIZE - 1)
      {
        debugInfo += ", ";
      }
    }
    Serial.println(debugInfo);

    if (haveSampleBufferSamplesReachedLimit(1, 2))
    {
      clearSampleBuffer();
      updateNoiseBuffer();

      Serial.println("Sensor haveSampleBufferSamplesReachedLimit: true");

      if (hasNoiseBufferReachedLimit(2))
      {
        Serial.println("Sensor hasNoiseBufferReachedLimit: true");

        clearNoiseBuffer();

        publishMessage(MQTT_TOPIC_NOISE, MQTT_TOPIC_NOISE_ON);

        Serial.println("Sensor LED on");
        digitalWrite(DIGITAL_PIN_LED, HIGH);

        xTimerStart(hasNoiseBufferReachedLimitTimer, 0);
      }
      else
      {
        Serial.println("Sensor LED on");
        digitalWrite(DIGITAL_PIN_LED, HIGH);

        xTimerStart(noiseDetectedTimer, 0);
      }
    }
  }

  delay(500);
}
