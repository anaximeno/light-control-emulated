#include <WiFi.h>
#include <PubSubClient.h>

#define CLIENT_ID "XaWokwiESP32Client"

#define ENABLED_LED '1'
#define DISABLED_LED '0'
#define MESSAGE_MAX_SIZE 128
#define BTN_DEBOUNCE_TIME 100
#define LIGHT_THRESOLD 0.15
#define LIGHTS_AUTO_ENABLE_INTERVAL 8000

const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* mqtt_server = "broker.emqx.io";
const int mqtt_port = 1883;
const char* topic_pub_sensor_light = "xa/wokwi/esp32/sensor/light";
const char* topic_pub_led_status = "xa/wokwi/esp32/led/status";
const char* topic_sub_led_blue_switch = "xa/wokwi/esp32/led/blue/switch";
const char* topic_sub_led_red_switch = "xa/wokwi/esp32/led/red/switch";
const char* topic_sub_led_green_switch = "xa/wokwi/esp32/led/green/switch";

const int blueLedPin = 4;
const int redLedPin = 19;
const int greenLedPin = 5;

static int blueLedState = 0;
static int redLedState = 0;
static int greenLedState = 0;

const int blueBtnPin = 26;
const int redBtnPin = 14;
const int greenBtnPin = 15;

static int lastBlueButtonState = HIGH;
static int lastRedButtonState = HIGH;
static int lastGreenButtonState = HIGH;

static int ledsAutoEnableRegister = 0;
static int ledsAutoDisableRegister = 0;

const int lightSensorPin = 34;
const float GAMMA = 0.7;
const float RL10 = 50;
const float maxLux = 13461;

WiFiClient espClient;
PubSubClient client(espClient);


void connectToWifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password, 6);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Msg arrived [");
  Serial.print(topic);
  Serial.print("]: ");

  if (strcmp(topic, topic_sub_led_blue_switch) == 0) {
    int blueLedExpectedState = ((char) payload[0]) == ENABLED_LED;

    Serial.print("Expected Blue Led State: ");
    Serial.println(blueLedExpectedState);

    digitalWrite(blueLedPin, blueLedExpectedState ? HIGH : LOW);

    blueLedState = blueLedExpectedState;
  } else if (strcmp(topic, topic_sub_led_red_switch) == 0) {
    int redLedExpectedState = ((char) payload[0]) == ENABLED_LED;

    Serial.print("Expected Red Led State: ");
    Serial.println(redLedExpectedState);

    digitalWrite(redLedPin, redLedExpectedState ? HIGH : LOW);

    redLedState = redLedExpectedState;
  } else if (strcmp(topic, topic_sub_led_green_switch) == 0) {
    int greenLedExpectedState = ((char) payload[0]) == ENABLED_LED;

    Serial.print("Expected Green Led State: ");
    Serial.println(greenLedExpectedState);

    digitalWrite(greenLedPin, greenLedExpectedState ? HIGH : LOW);

    greenLedState = greenLedExpectedState;
  }
}

void connectToMqtt() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT broker...");
    if (client.connect(CLIENT_ID)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void sendMessage(const char* topic, String message) {
  char msg[MESSAGE_MAX_SIZE];
  message.toCharArray(msg, MESSAGE_MAX_SIZE);
  // Serial.print("Publish message: ");
  // Serial.println(msg);
  client.publish(topic, msg);
}

void messageLedStatus(long timestamp, String color, int status) {
  String message = String(timestamp) + "," + String(color) + "," + String(status);
  sendMessage(topic_pub_led_status, message);
}

float verifyLuxIntensity(long timestamp, float intensity) {
  float voltage = intensity / 1024. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

  if (isnan(lux)) lux = 0;

  float percentLux = lux / maxLux;

  Serial.print("Lux: ");
  Serial.println(lux);

  String message = String(timestamp) + "," + String(lux) + "," + String(percentLux);
  sendMessage(topic_pub_sensor_light, message);

  return percentLux;
}

void checkButtonsClick(long timestamp) {
  int currentBlueButtonState = digitalRead(blueBtnPin);
  int currentRedButtonState = digitalRead(redBtnPin);
  int currentGreenButtonState = digitalRead(greenBtnPin);

  static long lastBlueBtnClickStamp = 0;
  static long lastRedBtnClickStamp = 0;
  static long lastGreenBtnClickStamp = 0;

  if (lastBlueButtonState == LOW &&
      currentBlueButtonState == HIGH &&
      (timestamp - lastBlueBtnClickStamp) > BTN_DEBOUNCE_TIME) {
    // ---
    lastBlueBtnClickStamp = timestamp;
    blueLedState = !blueLedState;
    digitalWrite(blueLedPin, blueLedState ? HIGH : LOW);
    messageLedStatus(timestamp, "blue", blueLedState);
    Serial.print("Click ");
    Serial.println(blueLedState);
  }

  if (lastRedButtonState == LOW &&
      currentRedButtonState == HIGH &&
      (timestamp - lastRedBtnClickStamp) > BTN_DEBOUNCE_TIME) {
    // ---
    lastRedBtnClickStamp = timestamp;
    redLedState = !redLedState;
    digitalWrite(redLedPin, redLedState ? HIGH : LOW);
    messageLedStatus(timestamp, "red", redLedState);
    Serial.print("Click ");
    Serial.println(redLedState);
  }

  if (lastGreenButtonState == LOW &&
      currentGreenButtonState == HIGH &&
      (timestamp - lastGreenBtnClickStamp) > BTN_DEBOUNCE_TIME) {
    // ---
    lastGreenBtnClickStamp = timestamp;
    greenLedState = !greenLedState;
    digitalWrite(greenLedPin, greenLedState ? HIGH : LOW);
    messageLedStatus(timestamp, "green", greenLedState);
    Serial.print("Click ");
    Serial.println(greenLedState);
  }

  lastBlueButtonState = currentBlueButtonState;
  lastRedButtonState = currentRedButtonState;
  lastGreenButtonState = currentGreenButtonState;
}

void changeAllLedsState(long timestamp, int state) {
  digitalWrite(blueLedPin, state ? HIGH : LOW);
  digitalWrite(redLedPin, state ? HIGH : LOW);
  digitalWrite(greenLedPin, state ? HIGH : LOW);
  blueLedState = state;
  redLedState = state;
  greenLedState = state;
  messageLedStatus(timestamp, "blue", state);
  messageLedStatus(timestamp, "red", state);
  messageLedStatus(timestamp, "green", state);
}

void autoEnableLights(long timestamp, float percentLux) {
  static long lastLightAutoEnableCheck = millis();

  if ((timestamp - lastLightAutoEnableCheck) > LIGHTS_AUTO_ENABLE_INTERVAL) {
    if ((percentLux < LIGHT_THRESOLD) && !ledsAutoEnableRegister) {
      ledsAutoEnableRegister = 1;
      ledsAutoDisableRegister = 0;
      Serial.print("Enabling Lights, LP= ");
      Serial.println(percentLux);
      changeAllLedsState(timestamp, 1);
    } else if ((percentLux >= LIGHT_THRESOLD) && !ledsAutoDisableRegister) {
      ledsAutoDisableRegister = 1;
      ledsAutoEnableRegister = 0;
      Serial.print("Disabling Lights, LP= ");
      Serial.println(percentLux);
      changeAllLedsState(timestamp, 0);
    }
    lastLightAutoEnableCheck = timestamp;
  }

}

void setup() {
  Serial.begin(9600);

  connectToWifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  connectToMqtt();

  client.subscribe(topic_sub_led_blue_switch);
  client.subscribe(topic_sub_led_red_switch);
  client.subscribe(topic_sub_led_green_switch);
  Serial.println("Subscribed to Led topics!");

  pinMode(blueLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(lightSensorPin, INPUT);
  pinMode(blueBtnPin, INPUT_PULLUP);
  pinMode(redBtnPin, INPUT_PULLUP);
  pinMode(greenBtnPin, INPUT_PULLUP);
  digitalWrite(blueLedPin, LOW);
  Serial.println("Pins Set.");
}

void loop() {
  if (!client.connected()) {
    connectToMqtt();
  }

  long now = millis();

  checkButtonsClick(now);

  static long lightSensorLastRead = 0;
  static long ledStatusLastSend = 0;

  if (now - ledStatusLastSend > 500) {
    ledStatusLastSend = now;
    messageLedStatus(now, "blue", blueLedState);
    messageLedStatus(now, "red", redLedState);
    messageLedStatus(now, "green", greenLedState);
  }

  if (now - lightSensorLastRead > 250) {
    lightSensorLastRead = now;
    float intensity = analogRead(lightSensorPin);
    float percentLux = verifyLuxIntensity(now, intensity);
    autoEnableLights(now, percentLux);
  }

  // ensure MQTT connection alive
  client.loop();
}