#include <WiFi.h>

#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

// Configuration WiFi
#define WLAN_SSID       ""
#define WLAN_PASS       ""

// Configuration Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""
// Numéro de broche de la LED interne de l'ESP32
const int ledPin = 2; // Remplacez 2 par le numéro de broche de votre ESP32 où est connectée la LED interne
const int ledPin2 = 15;


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Tempe");
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidite");
//Adafruit_MQTT_Publish pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure");
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/button");

Adafruit_BME680 bme;

void MQTT_connect();

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println(F("Adafruit MQTT demo"));

  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT); 


  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt.subscribe(&onoffbutton);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C pendant 150 ms
}

void loop() {
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(0))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
      int buttonState = atoi((char *)onoffbutton.lastread);

      if (strcmp((char *)onoffbutton.lastread, "ON") == 0) {
        digitalWrite(ledPin, HIGH); 
        Serial.println("LEDv is ON");
        digitalWrite(ledPin2, LOW);
        Serial.println("LEDr is OFF");
      } else if(strcmp((char *)onoffbutton.lastread, "OFF") == 0){
        digitalWrite(ledPin, LOW); 
        Serial.println("LEDv is OFF");
        digitalWrite(ledPin2, HIGH);
        Serial.println("LEDr is ON");
      }

    }
  }

  if (bme.performReading()) {
    float temp = bme.temperature;
    float humi = bme.humidity;
    float pres = bme.pressure;

    Serial.print(F("\nSending temperature: "));
    Serial.print(temp);
    Serial.print("°C");
    if (!temperature.publish(temp)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("  OK!"));
    }
  delay(1000);
    Serial.print(F("\nSending humidity: "));
    Serial.print(humi);
    Serial.print("%");
    if (!humidity.publish(humi)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F(" OK!"));
    }
  delay(1000);
    /*
    Serial.print(F("\nSending pressure: "));
    Serial.print(pres);
    Serial.print("Pa");
    if (!pressure.publish(pres)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F(" OK!"));
    }
  delay(100);*/
  }
  delay(3000);

}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1);
    }
  }
  Serial.println("MQTT Connected!");
}

Deuxième code qui permet de récupérer les données :
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <LiquidCrystal_I2C.h>  // Utilisez la bibliothèque LCD adaptée à votre écran

#define uS_TO_mS_FACTOR 1000ULL
#define TIME_TO_SLEEP 2000

// Configuration WiFi
#define WLAN_SSID ""
#define WLAN_PASS ""

// Configuration Adafruit IO
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME ""
#define AIO_KEY ""

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

Adafruit_MQTT_Subscribe temperature = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Tempe");
Adafruit_MQTT_Subscribe humidity = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Humidite");
//Adafruit_MQTT_Subscribe pressure = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Pressure");

// Configuration de l'écran LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adresse I2C et dimensions de l'écran

void MQTT_connect();

 RTC_DATA_ATTR float temp = -1;
 RTC_DATA_ATTR float humi = -1;
 RTC_DATA_ATTR float press = -1;

void setup() {
  Serial.begin(115200);
  delay(10);

  // Connexion WiFi
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Abonnement aux flux
  mqtt.subscribe(&temperature);
  mqtt.subscribe(&humidity);
  //mqtt.subscribe(&pressure);

  // Initialisation de l'écran LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 2);
  lcd.print("Temp: ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");

  lcd.setCursor(10, 2);
  lcd.print(temp);
  lcd.print("C");
  lcd.setCursor(10, 1);
  lcd.print(humi);
  lcd.print("%");
  //lcd.setCursor(0, 0);
  //lcd.print("Pressure: ");

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_mS_FACTOR);
}


void loop() {
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &temperature) {
      temp = atof((char *)temperature.lastread);
      Serial.println((char *)temperature.lastread);
    }
    //delay(100);
    if (subscription == &humidity) {
      humi = atof((char *)humidity.lastread);

    }  //delay(100);
       /*if (subscription == &pressure) {
      press = atof((char *)pressure.lastread);
      
    }delay(100);*/
  }
  lcd.setCursor(10, 2);
  lcd.print(temp);
  lcd.print("C");
  lcd.setCursor(10, 1);
  lcd.print(humi);
  lcd.print("%");
  /*lcd.setCursor(10, 0);
    lcd.print(press);
    lcd.print("Pa");*/
  esp_deep_sleep_start();
}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
    retries--;
    if (retries == 0) {
      while (1)
        ;
    }
  }
  Serial.println("MQTT Connected!");
}
