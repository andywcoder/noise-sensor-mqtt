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
//#define MQTT_HOST IPAddress(37, 187, 106, 16)
//#define MQTT_PORT 1883
#define MQTT_SENSOR_NAME "sensor1"
#define MQTT_TOPIC_STATE "stat/sensor1/state"
#define MQTT_TOPIC_STATE_READY "READY"
#define MQTT_TOPIC_STATE_ENABLED "ENABLED"
#define MQTT_TOPIC_STATE_DISABLED "DISABLED"
#define MQTT_TOPIC_NOISE "stat/sensor1/noise"
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

void publishReadyState()
{
  Serial.print("MQTT ");
  Serial.print(MQTT_TOPIC_STATE);
  Serial.print(" ");
  Serial.println(MQTT_TOPIC_STATE_READY);
  mqttClient.publish(MQTT_TOPIC_STATE, 0, true, MQTT_TOPIC_STATE_READY);
}

void handleMessage(char *topic, char *payload)
{
  if (strcmp(topic, MQTT_TOPIC_STATE) == 0)
  {
    if (strcmp(payload, MQTT_TOPIC_STATE_ENABLED) == 0)
    {
      Serial.print("Disable sensor");
      isEnabled = true;
    }
    else if (strcmp(payload, MQTT_TOPIC_STATE_DISABLED) == 0)
    {
      Serial.print("Enable sensor");
      isEnabled = false;
    }
  }
}

void noiseDetectedTimerCallback()
{
  Serial.println("LED off");
  digitalWrite(DIGITAL_PIN_LED, LOW);
}

void hasNoiseBufferReachedLimitTimerCallback()
{
  Serial.println("LED off");
  digitalWrite(DIGITAL_PIN_LED, LOW);

  Serial.print("MQTT ");
  Serial.print(MQTT_TOPIC_NOISE);
  Serial.print(" ");
  Serial.println(MQTT_TOPIC_NOISE_OFF);
  mqttClient.publish(MQTT_TOPIC_NOISE, 0, true, MQTT_TOPIC_NOISE_OFF);
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
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);

  uint16_t packetIdSub = mqttClient.subscribe(MQTT_TOPIC_STATE, 0);
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);

  publishReadyState();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);

  handleMessage(topic, payload);
}

void onMqttPublish(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
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
    debugInfo += "digitalSample: ";
    debugInfo += String(digitalSample);
    debugInfo += "\r\nanalogSample: ";
    debugInfo += String(analogSample);
    debugInfo += "\r\nanalogSampleVoltage: ";
    debugInfo += String(analogSampleVoltage, 3);
    debugInfo += "\r\nsampleBuffer: ";
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
    debugInfo += "\r\nnoiseBuffer: ";
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

      Serial.println("haveSampleBufferSamplesReachedLimit");

      if (hasNoiseBufferReachedLimit(2))
      {
        clearNoiseBuffer();

        Serial.print("MQTT ");
        Serial.print(MQTT_TOPIC_NOISE);
        Serial.print(" ");
        Serial.println(MQTT_TOPIC_NOISE_ON);
        mqttClient.publish(MQTT_TOPIC_NOISE, 0, true, MQTT_TOPIC_NOISE_ON);

        Serial.println("LED on");
        digitalWrite(DIGITAL_PIN_LED, HIGH);

        xTimerStart(hasNoiseBufferReachedLimitTimer, 0);
      }
      else
      {
        Serial.println("LED on");
        digitalWrite(DIGITAL_PIN_LED, HIGH);

        xTimerStart(noiseDetectedTimer, 0);
      }
    }
  }

  delay(500);
}
