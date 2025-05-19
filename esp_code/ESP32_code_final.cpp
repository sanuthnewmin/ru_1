#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>

// WiFi & MQTT Config
#define WIFI_SSID "enter your WIFI SSID"     
#define WIFI_PASSWORD "enter your WIFI password"  
#define MQTT_HOST "test.mosquitto.org"   
#define MQTT_PORT 1883

// Global MQTT client and timers for reconnection logic
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long lastPublish = 0;  // Timestamp to control publish interval
bool statusPublished = false;   // Tracks if "online" status was published

// Connects to the WiFi network
void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// Connects to the MQTT broker
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

// Called when MQTT successfully connects
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT broker");
  mqttClient.publish("factory/status", 2, false, "online");  //// Publish initial online status to topic `factory/status`
  statusPublished = true;
}

// Called when MQTT disconnects unexpectedly
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.print("Disconnected from MQTT. Reason: ");
  Serial.println(static_cast<int>(reason));
  xTimerStart(mqttReconnectTimer, 0);     // Attempt reconnection after delay
  statusPublished = false;
}

// Handles WiFi events like connected or disconnected
void onWiFiEvent(WiFiEvent_t event) {
  if (event == SYSTEM_EVENT_STA_GOT_IP) {
    Serial.println("WiFi connected");
    connectToMqtt();
  } else if (event == SYSTEM_EVENT_STA_DISCONNECTED) {
    Serial.println("WiFi disconnected");
    xTimerStop(mqttReconnectTimer, 0);   
    xTimerStart(wifiReconnectTimer, 0);   // Try reconnecting to WiFi
  }
}

// Publishes JSON data for 3 devices to MQTT topic `factory/all`
void publishAllDeviceData() {
  StaticJsonDocument<768> doc;   // Create a JSON document with 768 bytes of capacity

// Loop through 3 devices and generate random sensor values
  for (int i = 1; i <= 3; i++) {
    JsonObject dev = doc.createNestedObject("device" + String(i));
    dev["IR_Sensor"] = random(2000, 5000) / 100.0;
    dev["K_Type_Sensor"] = random(1000, 3000) / 100.0;
    dev["Current_Miller"] = random(3000, 6000) / 100.0;
    dev["Current_Mixer"] = random(1500, 4000) / 100.0;
  }

// Serialize JSON to string and publish to MQTT
  String payload;
  serializeJson(doc, payload);
  mqttClient.publish("factory/all", 2, true, payload.c_str());  // QoS 2, retain = true

  Serial.println("Published:");
  Serial.println(payload);
}

// Setup function runs once at boot
void setup() {
  Serial.begin(115200);
  delay(1000);    // Delay to allow Serial Monitor to initialize
  Serial.println(" ESP32 Booting...");

// Create timers for reconnecting to MQTT and WiFi
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, NULL,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, NULL,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWiFi));

  WiFi.onEvent(onWiFiEvent);                  //WIFI event handler
  mqttClient.onConnect(onMqttConnect);        //MQTT event handler
  mqttClient.onDisconnect(onMqttDisconnect);

// Set MQTT server address and port
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setClientId("ESP32-MQTT-ALL");    // MQTT client ID
  mqttClient.setKeepAlive(10);
  mqttClient.setCleanSession(true);            // No persistent session

  // Set Last Will and Testament message (sent if ESP32 disconnects unexpectedly)
  mqttClient.setWill("factory/status", 2, true, "offline");

 // Start WiFi connection
  connectToWiFi();
}

// Main loop runs continuously after setup
void loop() {
  if (WiFi.isConnected() && mqttClient.connected()) {            // Only publish if both WiFi and MQTT are connected
    if (!statusPublished) {
      mqttClient.publish("factory/status", 2, true, "online");   // If not yet published online status, do it now
      statusPublished = true;
    }
 
 // Publish sensor data every 5 seconds
    if (millis() - lastPublish > 5000) {
      lastPublish = millis();
      publishAllDeviceData();
    }
  } else {
    statusPublished = false;    // Reset status flag if disconnected
  }
}
