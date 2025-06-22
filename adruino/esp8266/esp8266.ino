#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750.h>
#include <Servo.h>

// Wi-Fi credentials
const char* ssid = "International University";
const char* password = "";

// MQTT broker details
const char* mqtt_server = "silentprince839.cloud.shiftr.io";
const int mqtt_port = 1883;
const char* mqtt_user = "silentprince839";
const char* mqtt_pass = "pOSBE5CfXH1Oaz6z";
const char* mqtt_client_id = "ESP8266Client";

// GPIO
const int buttonPins[] = {D1, D2, D0}; // Fan button, LED button, Servo button
const int fanPin = D5;                 // Fan output
const int ledPin = D6;                 // LED output
const int dhtPin = D7;                 // DHT11 sensor pin
const int soundPin = A0;               // Sound sensor analog pin
const int servoPin = D8;               // Servo output pin

// I2C pins for BH1750
#define SDA_PIN D4
#define SCL_PIN D3

// DHT11 setup
#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);

// BH1750 setup
BH1750 lightSensor;

// Servo setup
Servo myServo;

// Timing for sensor readings
unsigned long lastSensorRead = 0;
const long sensorInterval = 8000; // 3 seconds

// MQTT status
bool mqttConnected = false;

WiFiClient espClient;
PubSubClient client(espClient);

bool fanState = false;
bool ledState = false;
bool servoState = false; // Servo on/off state
bool prevButtonState[3] = {HIGH, HIGH, HIGH}; // Fan, LED, Servo buttons
int servoAngle = 90; // Initial servo position

void setup_wifi() {
  Serial.printf("Connecting to %s...", ssid);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi. Restarting...");
    ESP.restart();
  }
}

void publishState(const char* topic, bool state) {
  if (client.publish(topic, state ? "on" : "off")) {
    Serial.printf("Published %s: %s\n", topic, state ? "on" : "off");
  } else {
    Serial.printf("Failed to publish %s\n", topic);
    mqttConnected = false; // Trigger reconnection
  }
}

void publishServoAngle(int angle) {
  char angleStr[4];
  itoa(angle, angleStr, 10);
  if (client.publish("servo_angle", angleStr)) {
    Serial.printf("Published servo_angle: %s\n", angleStr);
  } else {
    Serial.println("Failed to publish servo angle");
    mqttConnected = false;
  }
}

float readSoundLevel() {
  const int sampleWindow = 100; // 100 ms
  unsigned long startMillis = millis();
  float peakToPeak = 0;
  int signalMax = 0;
  int signalMin = 1024;

  while (millis() - startMillis < sampleWindow) {
    int sample = analogRead(soundPin);
    Serial.printf("Raw ADC value: %d\n", sample);
    if (sample < 1024 && sample >= 0) {
      if (sample > signalMax) signalMax = sample;
      else if (sample < signalMin) signalMin = sample;
    }
  }

  peakToPeak = signalMax - signalMin;
  Serial.printf("signalMax: %d, signalMin: %d, peakToPeak: %.2f\n", signalMax, signalMin, peakToPeak);
  float db = 20 * log10(peakToPeak / 10.0);
  if (db < 0) db = 0;
  return db;
}

void publishSensorReadings() {
  // Read DHT11
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  // Read sound level
  float sound = readSoundLevel();
  // Read light intensity
  float light = lightSensor.readLightLevel();
  if (light < 0) {
    Serial.println("Invalid light intensity reading!");
    light = 0; // Handle invalid BH1750 reading
  }

  // Validate DHT readings
  if (isnan(temp) || isnan(hum)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    char tempStr[8];
    char humStr[8];
    dtostrf(temp, 6, 2, tempStr);
    dtostrf(hum, 6, 2, humStr);

    // Display all sensor readings on Serial Monitor
    Serial.println("-------------------");
    Serial.printf("DHT11 Readings:\n");
    Serial.printf("Temperature: %s °C\n", tempStr);
    Serial.printf("Humidity: %s %%\n", humStr);
    Serial.printf("Sound Level: %.2f dB\n", sound);
    Serial.printf("Light Intensity: %.2f lux\n", light);
    Serial.println("-------------------");

    // Publish temperature
    if (client.publish("temperature", tempStr)) {
      Serial.printf("Published temperature: %s\n", tempStr);
    } else {
      Serial.println("Failed to publish temperature");
      mqttConnected = false;
    }

    // Publish humidity
    if (client.publish("humidity", humStr)) {
      Serial.printf("Published humidity: %s\n", humStr);
    } else {
      Serial.println("Failed to publish humidity");
      mqttConnected = false;
    }
  }

  // Publish sound level
  char soundStr[8];
  dtostrf(sound, 6, 2, soundStr);
  if (client.publish("sound", soundStr)) {
    Serial.printf("Published sound: %s\n", soundStr);
  } else {
    Serial.println("Failed to publish sound");
    mqttConnected = false;
  }

  // Publish light intensity
  char lightStr[8];
  dtostrf(light, 6, 2, lightStr);
  if (client.publish("light_intensity", lightStr)) {
    Serial.printf("Published light intensity: %s\n", lightStr);
  } else {
    Serial.println("Failed to publish light intensity");
    mqttConnected = false;
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  Serial.printf("Received %s: %s\n", topic, message.c_str());

  if (String(topic) == "fan") {
    fanState = (message == "on");
    digitalWrite(fanPin, fanState ? HIGH : LOW);
  } else if (String(topic) == "light") {
    ledState = (message == "on");
    digitalWrite(ledPin, ledState ? HIGH : LOW);
  } else if (String(topic) == "servo") {
    if (message == "on" || message == "off") {
      servoState = (message == "on");
      if (!servoState) {
        servoAngle = 0;
        myServo.write(servoAngle);
        publishServoAngle(servoAngle);
      }
      publishState("servo_state", servoState);
      Serial.println("Servo state set to " + String(servoState ? "on" : "off") + " via MQTT");
    } else {
      int angle = message.toInt();
      if (servoState && message.length() > 0 && angle >= 0 && angle <= 180) {
        servoAngle = angle;
        myServo.write(servoAngle);
        publishServoAngle(servoAngle);
        Serial.println("Servo set to " + String(servoAngle) + " degrees via MQTT");
      } else {
        Serial.println("Error: Invalid servo angle or servo is off. Must be 0-180.");
      }
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP8266Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe("fan");
      client.subscribe("light");
      client.subscribe("servo");
      mqttConnected = true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(". Retrying in 5 seconds...");
      delay(5000); // Wait 5 seconds before retrying
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow serial to stabilize
  Serial.println("ESP8266 Starting...");
  Serial.println("Servo Control - Enter angle (0-180 degrees) in Serial Monitor:");
  Serial.println("Press D0 button to toggle servo on/off");

  // Initialize I2C for BH1750
  Wire.begin(SDA_PIN, SCL_PIN);
  if (lightSensor.begin()) {
    Serial.println("BH1750 initialized successfully");
  } else {
    Serial.println("Failed to initialize BH1750! Check wiring or I2C address.");
  }

  // Initialize servo
  myServo.attach(servoPin);
  myServo.write(servoAngle); // Set initial position to 90 degrees
  Serial.println("Servo initialized on pin " + String(servoPin));
  Serial.println("Servo set to " + String(servoAngle) + " degrees");

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(fanPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPins[0], INPUT_PULLUP); // Fan button
  pinMode(buttonPins[1], INPUT_PULLUP); // LED button
  pinMode(buttonPins[2], INPUT);        // Servo button (D0, no internal pull-up)
  pinMode(soundPin, INPUT);             // Sound sensor pin

  digitalWrite(fanPin, LOW);
  digitalWrite(ledPin, LOW);

  dht.begin();
  delay(2000); // Allow DHT sensor to stabilize
  Serial.println("Setup complete. Monitoring sensors...");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Attempting to reconnect...");
    setup_wifi();
  }
  if (!client.connected() && mqttConnected) {
    mqttConnected = false;
    Serial.println("MQTT disconnected. Attempting to reconnect...");
  }
  if (!mqttConnected) {
    reconnect();
  }
  client.loop();

  // Fan toggle
  bool currentFanBtn = digitalRead(buttonPins[0]);
  if (prevButtonState[0] == HIGH && currentFanBtn == LOW) {
    fanState = !fanState;
    digitalWrite(fanPin, fanState ? HIGH : LOW);
    publishState("fan", fanState);
    delay(200);
  }
  prevButtonState[0] = currentFanBtn;

  // LED toggle
  bool currentLedBtn = digitalRead(buttonPins[1]);
  if (prevButtonState[1] == HIGH && currentLedBtn == LOW) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState ? HIGH : LOW);
    publishState("light", ledState);
    delay(200);
  }
  prevButtonState[1] = currentLedBtn;

  // Servo toggle
  bool currentServoBtn = digitalRead(buttonPins[2]);
  if (prevButtonState[2] == HIGH && currentServoBtn == LOW) {
    servoState = !servoState;
    if (!servoState) {
      servoAngle = 0;
      myServo.write(servoAngle);
      publishServoAngle(servoAngle);
    }
    publishState("servo_state", servoState);
    Serial.println("Servo state toggled to " + String(servoState ? "on" : "off") + " via button");
    delay(200);
  }
  prevButtonState[2] = currentServoBtn;

  // Servo control via Serial
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove whitespace
    int angle = input.toInt();
    if (servoState && input.length() > 0 && angle >= 0 && angle <= 180) {
      servoAngle = angle;
      myServo.write(servoAngle);
      publishServoAngle(servoAngle);
      Serial.println("Servo set to " + String(servoAngle) + " degrees via Serial");
    } else {
      Serial.println("Error: Invalid input or servo is off. Enter a number (0-180)");
    }
  }

  // Sensor readings
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorRead >= sensorInterval) {
    publishSensorReadings();
    lastSensorRead = currentMillis;
  }
}