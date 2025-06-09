#include <LiquidCrystal.h>
#include <dht11.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <RTC.h>
#include <Timezone.h>
#include <PubSubClient.h>

// MQ‑2 variables
#define MQ2PIN A0
const float RL = 5000.0;
float Ro = 0.0;

// Button variables
const int buttonPin = 9;
int prevButtonState = LOW;
int lcdMode = 0;

// DHT11 variables
const int dhtPin = 8;
dht11 DHT11;
float temperature = 0.0;
float humidity = 0.0;

// LCD initialization
typedef unsigned long ul;
const int LCD_ROWS = 2;
const int LCD_COLS = 16;
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Fan and Alarm
String fanMode = "auto";
String alarmMode = "auto";
const int fanPin = 12;
const int alarmPin = 13;
int fanState = 0;
int alarmState = 0;

// Timing variables
ul lastUpdateTime = 0;
const ul updateInterval = 2000;
ul lastWifiReconnect = 0;
const ul wifiReconnectInterval = 5000;
ul lastMqttReconnect = 0;
const ul mqttReconnectInterval = 2000;
ul lastNtpSync = 0;
const ul ntpSyncInterval = 300000;
ul lastMqttPublish = 0;
const ul mqttPublishInterval = 10000;
ul lastDhtUpdate = 0;
const ul dhtUpdateInterval = 1500;

// Wi‑Fi credentials
const char* ssid = "POCO";
const char* password = "1234567890";

// Clients
RTCTime rtcTime;
WiFiClient mqttClient;
WiFiUDP ntpUdp;
NTPClient timeClient(ntpUdp, "europe.pool.ntp.org", 0, 60000);
PubSubClient client(mqttClient);

// Timezone rules
TimeChangeRule CEST = {"CEST", Last, Sun, Mar, 2, 120};
TimeChangeRule CET  = {"CET",  Last, Sun, Oct, 3, 60};
Timezone CE(CEST, CET);

// Data structs
struct MQ2Data {
  int rawValue;
  float Rs;
  float ratio;
  float co_ppm;
  String co_level;
  float smoke_ppm;
  String smoke_level;
};


// MQTT parameters
const char* mqtt_server = "servermqtt.my.to";

// fan alarm tresholds
int tempThreshold  = 30;   // temperature threshold
int humThreshold   = 60;   // humidity threshold
int smokeThreshold = 300;  // smoke ppm threshold
int coThreshold    = 300;  // CO ppm threshold

// Topics data
const char* TOPIC_DHT11_TEMP      = "projekt_iiot/DHT11/temperature";
const char* TOPIC_DHT11_HUM       = "projekt_iiot/DHT11/humidity";

const char* TOPIC_MQ2_CO_PPM      = "projekt_iiot/MQ2/co/ppm";
const char* TOPIC_MQ2_CO_LEVEL    = "projekt_iiot/MQ2/co/level";
const char* TOPIC_MQ2_SMOKE_PPM   = "projekt_iiot/MQ2/smoke/ppm";
const char* TOPIC_MQ2_SMOKE_LEVEL = "projekt_iiot/MQ2/smoke/level";

// Topics nodered settings
const char* TOPIC_FAN_STATUS      = "projekt_iiot/fan/out";
const char* TOPIC_ALARM_STATUS    = "projekt_iiot/alarm/out";

// Topics local settings
const char* TOPIC_STATION_FAN_MODE     = "projekt_iiot/fan/mode";
const char* TOPIC_STATION_FAN_STATUS   = "projekt_iiot/fan/in";
const char* TOPIC_STATION_ALARM_MODE   = "projekt_iiot/alarm/mode";
const char* TOPIC_STATION_ALARM_STATUS = "projekt_iiot/alarm/in";

// Topics nodered settings
const char* TOPIC_TEMP_THRESHOLD  = "projekt_iiot/fan/tempthreshold";
const char* TOPIC_HUM_THRESHOLD   = "projekt_iiot/fan/humthreshold";
const char* TOPIC_SMOKE_THRESHOLD = "projekt_iiot/alarm/smokethreshold";
const char* TOPIC_CO_THRESHOLD    = "projekt_iiot/alarm/cothreshold";

// Topics local settings
const char* TOPIC_SETTINGS_TEMP_THRESHOLD  = "projekt_iiot/settings/tempthreshold";
const char* TOPIC_SETTINGS_HUM_THRESHOLD   = "projekt_iiot/settings/humthreshold";
const char* TOPIC_SETTINGS_SMOKE_THRESHOLD = "projekt_iiot/settings/smokethreshold";
const char* TOPIC_SETTINGS_CO_THRESHOLD    = "projekt_iiot/settings/cothreshold";


// Function prototypes
float calibrateRo(int pin, int samples = 50, int delayTime = 500);
MQ2Data readMQ2Sensor(int analogPin, float RL, float Ro);
void printSensorDataOnSerial(const MQ2Data &data);
void printDHT11DataOnSerial();
bool buttonPressed();
void printOnLcd(int mode, const MQ2Data &mq2, float temperature, float humidity, const String &ntpData);
void printTextOnLcd(const String &top, const String &bot);
void setRTCFromNTP();
bool connectToWiFi(int maxAttempts = 5, int delayMs = 500);
void initRtc();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnectMqtt();
void updateMqtt(const MQ2Data &mq2, float temperature, float humidity);
void publishSettings();
String formatDate(const RTCTime &t);
String formatTime(const RTCTime &t);

// ------------------------------------------------------------------
void setup() {
    Serial.begin(9600);
    initRtc();
    connectToWiFi();

    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);

    timeClient.begin();
    timeClient.update();
    if (timeClient.isTimeSet()) setRTCFromNTP();

    lcd.begin(LCD_COLS, LCD_ROWS);
    printTextOnLcd("MQ2 is", "Warming up...");
    delay(30000); // 30 sekund podgrzewania MQ-2

    Ro = calibrateRo(MQ2PIN, 50, 200);
    delay(500);
    printTextOnLcd("Ro val: ", String(Ro, 1) + "kohm");
    delay(3000);
    printTextOnLcd("Monitoring", "started!");
    delay(2000);
    lcd.clear();

    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(fanPin, OUTPUT);
    pinMode(alarmPin, OUTPUT);
}

void loop() {
    client.loop();
    // Wi‑Fi auto‑reconnect
    if (WiFi.status() != WL_CONNECTED && millis() - lastWifiReconnect >= wifiReconnectInterval) {
        connectToWiFi();
        lastWifiReconnect = millis();
    }
    // MQTT auto‑reconnect
    if (!client.connected() && millis() - lastMqttReconnect >= mqttReconnectInterval) {
        reconnectMqtt();
    }

    // Read time
    RTC.getTime(rtcTime);
    String ntpData = formatDate(rtcTime) + " " + formatTime(rtcTime);

    // Read DHT11
    if(millis() - lastDhtUpdate >= dhtUpdateInterval){
        int dhtData = DHT11.read(dhtPin);
        temperature = (float)DHT11.temperature;
        humidity = (float)DHT11.humidity; 
        lastDhtUpdate = millis();
    }

    // Read MQ‑2
    MQ2Data mq2 = readMQ2Sensor(MQ2PIN, RL, Ro);

    // Fan control
    if (fanMode == "manual") {
        digitalWrite(fanPin, fanState ? HIGH : LOW);
    } else {
        bool autoFanState = (temperature > tempThreshold || humidity > humThreshold);
        digitalWrite(fanPin, autoFanState ? HIGH : LOW);
    }

    // Alarm control
    if (alarmMode == "manual") {
        digitalWrite(alarmPin, alarmState ? HIGH : LOW);
    } else {
        bool autoAlarmState = (mq2.smoke_ppm > smokeThreshold || mq2.co_ppm > coThreshold);
        digitalWrite(alarmPin, autoAlarmState ? HIGH : LOW);
    }

    // Handle button
    if (buttonPressed()) {
        lcdMode = (lcdMode + 1) % 3;
        printOnLcd(lcdMode, mq2, temperature, humidity,ntpData);
    }

    // Periodic update
    if (millis() - lastUpdateTime >= updateInterval) {
        printOnLcd(lcdMode, mq2, temperature, humidity,ntpData);
        
        lastUpdateTime = millis();
    }
    if (millis() - lastMqttPublish > mqttPublishInterval){
        publishSettings();
        updateMqtt(mq2, temperature, humidity);
        printSensorDataOnSerial(mq2);
        printDHT11DataOnSerial();
    }


    // Periodic NTP sync
    if (WiFi.status() == WL_CONNECTED && millis() - lastNtpSync > ntpSyncInterval) {
        if (timeClient.update()) setRTCFromNTP();
            lastNtpSync = millis();
    }
}

// ------------------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String topicStr = String(topic);
    String msg;

    for (unsigned int i = 0; i < length; ++i)
        msg += (char)payload[i];

    msg.trim();
    msg.toLowerCase();

    Serial.print("Message arrived [");
    Serial.print(topicStr);
    Serial.print("] ");
    Serial.println(msg);

    // Fan control topics
    if (topicStr == TOPIC_FAN_STATUS) {
        if (msg == "auto" || msg == "manual") {
            fanMode = msg;
            Serial.println("Fan mode set to: " + fanMode);
        } else if (fanMode == "manual" && (msg == "on" || msg == "off")) {
            fanState = (msg == "on") ? 1 : 0;
            Serial.println("Fan manually turned " + msg);
        }
    } 

    // Alarm control topics
    else if (topicStr == TOPIC_ALARM_STATUS) {
        if (msg == "auto" || msg == "manual") {
            alarmMode = msg;
            Serial.println("Alarm mode set to: " + alarmMode);
        } else if (alarmMode == "manual" && (msg == "on" || msg == "off")) {
            alarmState = (msg == "on") ? 1 : 0;
            Serial.println("Alarm manually turned " + msg);
        }
    }

    // Threshold topics
    else if (topicStr == TOPIC_TEMP_THRESHOLD) {
        int val = msg.toInt();
        if (val > 0) {
            tempThreshold = val;
            Serial.println("Temperature threshold set to: " + String(tempThreshold));
        }
    }
    else if (topicStr == TOPIC_HUM_THRESHOLD) {
        int val = msg.toInt();
        if (val > 0) {
            humThreshold = val;
            Serial.println("Humidity threshold set to: " + String(humThreshold));
        }
    }
    else if (topicStr == TOPIC_SMOKE_THRESHOLD) {
        int val = msg.toInt();
        if (val > 0) {
            smokeThreshold = val;
            Serial.println("Smoke threshold set to: " + String(smokeThreshold));
        }
    }
    else if (topicStr == TOPIC_CO_THRESHOLD) {
        int val = msg.toInt();
        if (val > 0) {
            coThreshold = val;
            Serial.println("CO threshold set to: " + String(coThreshold));
        }
    }
}

void reconnectMqtt() {
    String clientId = "Arduino-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
        Serial.println("MQTT connected");

        // Subscribe to control topics
        client.subscribe(TOPIC_FAN_STATUS);
        client.subscribe(TOPIC_ALARM_STATUS);

        // Subscribe to threshold update topics
        client.subscribe(TOPIC_TEMP_THRESHOLD);
        client.subscribe(TOPIC_HUM_THRESHOLD);
        client.subscribe(TOPIC_SMOKE_THRESHOLD);
        client.subscribe(TOPIC_CO_THRESHOLD);

        // Publikacja settings po reconnect
        publishSettings();
    } else {
        Serial.print("MQTT failed, rc=");
        Serial.println(client.state());
    }
    lastMqttReconnect = millis();
}

void updateMqtt(const MQ2Data &mq2, float temperature, float humidity) {
    if (!client.connected()) return;

    char buf[16];
    dtostrf(temperature, 4, 1, buf);
    client.publish(TOPIC_DHT11_TEMP, buf);
    dtostrf(humidity, 4, 0, buf);
    client.publish(TOPIC_DHT11_HUM,  buf);

    dtostrf(mq2.co_ppm, 6, 1, buf);
    client.publish(TOPIC_MQ2_CO_PPM, buf);
    client.publish(TOPIC_MQ2_CO_LEVEL, mq2.co_level.c_str());

    dtostrf(mq2.smoke_ppm, 6, 1, buf);
    client.publish(TOPIC_MQ2_SMOKE_PPM, buf);
    client.publish(TOPIC_MQ2_SMOKE_LEVEL, mq2.smoke_level.c_str());
    lastMqttPublish = millis();
}

void publishSettings(){
    if (!client.connected()) return;

    // Publish current mode and state
    client.publish(TOPIC_STATION_FAN_MODE, fanMode.c_str());
    client.publish(TOPIC_STATION_FAN_STATUS, fanState ? "on" : "off");

    client.publish(TOPIC_STATION_ALARM_MODE, alarmMode.c_str());
    client.publish(TOPIC_STATION_ALARM_STATUS, alarmState ? "on" : "off");

    // Publish current threshold values on settings topic (retain=true)
    client.publish(TOPIC_SETTINGS_TEMP_THRESHOLD, String(tempThreshold).c_str(), true);
    client.publish(TOPIC_SETTINGS_HUM_THRESHOLD, String(humThreshold).c_str(), true);
    client.publish(TOPIC_SETTINGS_SMOKE_THRESHOLD, String(smokeThreshold).c_str(), true);
    client.publish(TOPIC_SETTINGS_CO_THRESHOLD, String(coThreshold).c_str(), true);
    lastMqttPublish = millis();
}

void initRtc() {
    RTC.begin();
    RTC.getTime(rtcTime);
    if (rtcTime.getYear() < 2020) {
        RTCTime def(1, Month::JANUARY, 2024, 0, 0, 0, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_INACTIVE);
        RTC.setTime(def);
        rtcTime = def;
    }
}

bool connectToWiFi(int maxAttempts, int delayMs) {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts) {
        delay(delayMs);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" Connected!");
        lastWifiReconnect = millis();
        delay(500);
        return true;
    }
    Serial.println(" Failed to connect.");
    delay(500);
    return false;
}

bool buttonPressed() {
    int reading = digitalRead(buttonPin);
    if (reading == HIGH && prevButtonState == LOW) {
        delay(50);
        prevButtonState = reading;
        return true;
    }
    prevButtonState = reading;
    return false;
}

void printOnLcd(int mode, const MQ2Data &mq2, float temperature, float humidity, const String &ntpData) {
  lcd.clear();
  switch (mode) {
        case 0: {
        printTextOnLcd("Temp: " + String(temperature, 1) + (char)223 + "C",
                       "Hum:  " + String(humidity, 0) + "%");
        break;
        }
        case 1: {
        printTextOnLcd("CO:    " + mq2.co_level.substring(0,8),
                       "Smoke: " + mq2.smoke_level.substring(0,8));
        break;
        }
        case 2: {
        printTextOnLcd("Date:" + ntpData.substring(0,10),
                       "Time:" + ntpData.substring(11));
        break;
        }
        default:
        printTextOnLcd("Invalid mode"," ");
  }
}

void printTextOnLcd(const String &top, const String &bot) {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print(top);
    lcd.setCursor(0,1); lcd.print(bot);
}

void printDHT11DataOnSerial() {
    Serial.print(F("Humidity (%): ")); Serial.print((float)DHT11.humidity,2);
    Serial.print(F("\tTemperature (C): ")); Serial.println((float)DHT11.temperature,2);
}

void printSensorDataOnSerial(const MQ2Data &d) {
    Serial.println(F("===================================="));
    Serial.print(F("Raw ADC Value: ")); Serial.println(d.rawValue);
    Serial.print(F("Sensor Resistance (Rs): ")); Serial.print(d.Rs,1); Serial.println(F(" ohm"));
    Serial.print(F("CO ppm: ")); Serial.print(d.co_ppm,1); Serial.print(F(" | Level: ")); Serial.println(d.co_level);
    Serial.print(F("Smoke ppm: ")); Serial.print(d.smoke_ppm,1); Serial.print(F(" | Level: ")); Serial.println(d.smoke_level);
}

float calibrateRo(int pin, int samples, int delayTime) {
    float sum = 0;

    for (int i = 0; i < samples; i++) {
        int a = analogRead(pin);       // surowy odczyt z czujnika (0–1023)
        float v = (a / 1023.0) * 5.0;  // przeliczenie na napięcie (Vout)
        sum += RL * ((5.0 - v) / v);   // obliczanie Rs
        delay(delayTime);
    }

    return (sum / samples) / 9.5;      // uśrednienie i przeliczenie Ro (zakładamy Rs/Ro = 9.5)
}

MQ2Data readMQ2Sensor(int pin, float RL, float Ro) {
    MQ2Data d;
    d.rawValue = analogRead(pin);                   // surowy odczyt (0–1023)
    float Vout = (d.rawValue / 1023.0) * 5.0;       // przeliczenie na napięcie (Vout)
    d.Rs = RL * ((5.0 - Vout) / Vout);              // obliczenie rezystancji czujnika Rs
    d.ratio = d.Rs / Ro;                             // stosunek Rs/Ro
    
    // obliczenie ppm dla CO i dymu wg wzorów kalibracyjnych
    d.co_ppm = 610.94 * pow(d.ratio, -2.105);
    d.smoke_ppm = 2449.06 * pow(d.ratio, -2.723);

    // funkcja pomocnicza do wyznaczenia poziomu zagrożenia na podstawie progów
    auto lvl = [](float p, const int thresholds[], const char* names[]) {
        for (int i = 0; i < 3; i++)
            if (p <= thresholds[i]) return String(names[i]);
        return String(names[3]);
    };

    // progi dla CO (ppm)
    const int th_co[] = {50, 100, 200};
    const char* n_co[] = {"Normal", "Moderate", "High", "VeryHigh"};

    // progi dla dymu (ppm)
    const int th_sm[] = {100, 300, 700};
    const char* n_sm[] = {"Normal", "Moderate", "High", "VeryHigh"};

    d.co_level = lvl(d.co_ppm, th_co, n_co);         // przypisanie poziomu dla CO
    d.smoke_level = lvl(d.smoke_ppm, th_sm, n_sm);   // przypisanie poziomu dla dymu
    
    return d;
}

void setRTCFromNTP(){
    ul e = timeClient.getEpochTime();
    time_t lt = CE.toLocal(e);
    tm *t = gmtime(&lt);
    RTCTime rt( t->tm_mday, 
                Month(t->tm_mon+1), 
                t->tm_year+1900,
                t->tm_hour, 
                t->tm_min, 
                t->tm_sec,
                DayOfWeek(t->tm_wday? t->tm_wday : 7),
                SaveLight::SAVING_TIME_INACTIVE
            );
    RTC.setTime(rt);
    Serial.println("RTC synced");
}

String formatDate(const RTCTime &t){
    char b[11];
    snprintf(b,11,"%02d/%02d/%04d",t.getDayOfMonth(),int(t.getMonth()),t.getYear());
    return String(b);
}  
String formatTime(const RTCTime &t){
    char b[6];
    snprintf(b,6,"%02d:%02d",t.getHour(),t.getMinutes());
    return String(b);
}  
