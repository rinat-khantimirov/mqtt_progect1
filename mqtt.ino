// Светодиод подлкючен к 5 пину
// Датчик температуры ds18b20 к 2 пину
#include <EEPROM.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire ds(2);
DHT dht(7, DHT11);
DHT dht1(8, DHT11);
DallasTemperature sensors(&ds);
DeviceAddress sensorAddress;
int h2, h1, t2, t1, t3;
bool air1 = true, air2 = false;
int h_max = 70;
int h1_max = 70;
int tm = 1000;


const char *ssid = "RJ_love4ever"; // Имя вайфай точки доступа
const char *pass = "51213520"; // Пароль от точки доступа

const char *mqtt_server = "m15.cloudmqtt.com"; // Имя сервера MQTT
const int mqtt_port =   16204; // Порт для подключения к серверу MQTT
const char *mqtt_user = "tdetizjs"; // Логи от сервер
const char *mqtt_pass = " p1XWPyTQ_KQD"; // Пароль от сервера

#define BUFFER_SIZE 100



// Функция получения данных от сервера

void callback(const MQTT::Publish& pub)
{
  Serial.print(pub.topic()); // выводим в сериал порт название топика
  Serial.print(" => ");
  Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных

  String payload = pub.payload_string();

  if (String(pub.topic()) == "h_max") // проверяем из нужного ли нам топика пришли данные
  {
    h_max = payload.toInt(); // преобразуем полученные данные в тип integer
  }
  if (String(pub.topic()) == "h1_max") // проверяем из нужного ли нам топика пришли данные
  {
    h1_max = payload.toInt(); // преобразуем полученные данные в тип integer
  }
}

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

void setup() {

  sensors.begin();
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  sensors.setResolution(sensorAddress, 8);
  lcd.init();
  lcd.backlight();
  delay(1000);
  /* h_max = EEPROM.read(0); раскоментировать в рабочей програме.
    h1_max = EEPROM.read(2);
  */
}

void loop() {
  // подключаемся к wi-fi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, pass);

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("WiFi connected");
  }

  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect("arduinoClient2")
                         .set_auth(mqtt_user, mqtt_pass))) {
        Serial.println("Connected to MQTT server");
        client.set_callback(callback);
        client.subscribe("h_max");
        client.subscribe("h1_max");
      } else {
        Serial.println("Could not connect to MQTT server");
      }
    }

    if (client.connected()) {
      client.loop();
      TempSend();
    }

  }
  h2 = dht.readHumidity();
  t2 = dht.readTemperature();
  h1 = dht1.readHumidity();
  t1 = dht1.readTemperature();
  if (!sensors.getAddress(sensorAddress, 0))
    sensors.requestTemperatures();
  t3 = sensors.getTempC(sensorAddress);
  int  delta1 =  h_max - h1;
  int  delta2 =  h1_max - h2;
  if (h2 != 0) {
    if (delta2 <= 0) {
      air2 = false;
      digitalWrite(6, HIGH);
    }
    else if (delta2 >= 10) {
      air2 = true;
      digitalWrite(6, HIGH);
    }
  }
  else {
    air2 = false;
    digitalWrite(6, LOW);
  }
  if (h1 != 0) {
    if (delta1 <= 0) {
      air1 = false;
      digitalWrite(9, LOW);
    }
    else if (delta1 >= 10) {
      air1 = true;
      digitalWrite(9, HIGH);
    }
  }
  else {
    air1 = false;
    digitalWrite(9, LOW);
  }
  lcd.setCursor(0, 0); lcd.print(t1); lcd.print("\xDF");
  lcd.setCursor(0, 1); lcd.print(h1); lcd.print("%");
  lcd.setCursor(5, 0); lcd.print(t2); lcd.print("\xDF");
  lcd.setCursor(5, 1); lcd.print(h2); lcd.print("%");
  lcd.setCursor(10, 0); lcd.print(t3); lcd.print("\xDF");
  Serial.print("Влажность DHT22: "); Serial.print(h2); Serial.print(" %   ");
  Serial.print("Температура DHT22: "); Serial.print(t2); Serial.print(" °C    ");
  Serial.print("Влажность DHT11: "); Serial.print(h1); Serial.print(" %   ");
  Serial.print("Температура DHT11: "); Serial.print(t1); Serial.print(" °C   ");
  Serial.print ("Температура DS18b20= "); Serial.print(t3); Serial.println(" °C");
  sensors.requestTemperatures();
} // конец основного цикла

// Функция отправки показаний с термодатчика
void TempSend() {
  if (tm == 0)
  {

    StaticJsonBuffer<300> JSONbuffer;
    JsonObject& JSONencoder = JSONbuffer.createObject();

    JSONencoder["t1"] = String(t1);
    JSONencoder["t2"] = String(t2);
    JSONencoder["t3"] = String(t3);
    JSONencoder["h1"] = String(h1);
    JSONencoder["h2"] = String(h2);
    JSONencoder["air1"] = String(air1);
    JSONencoder["air2"] = String(air2);
    JSONencoder["h_max"] = String(h_max);
    JSONencoder["h1_max"] = String(h1_max);

    char JSONmessageBuffer[200];
    JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
    Serial.println("Sending message to MQTT topic..");
    Serial.println(JSONmessageBuffer);

    if (client.publish("DATA", JSONmessageBuffer) == true) {
      Serial.println("Success sending message");
    } else {
      Serial.println("Error sending message");
    }
    sensors.requestTemperatures(); // от датчика получаем значение температуры
    tm = 1000; // пауза меду отправками значений температуры коло 10 секунд
  }
  tm--;
  delay(10);
}
