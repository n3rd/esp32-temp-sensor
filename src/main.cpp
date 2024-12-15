#include <Arduino.h>
#include <esp_adc_cal.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "secrets.h"
#include "config.h"

/************************* BME280 Setup *********************************/

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

/************ Global State (you don't need to change this!) ******************/

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
Adafruit_MQTT_Publish bmeTemperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme280-temperature");
Adafruit_MQTT_Publish bmePressureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme280-pressure");
Adafruit_MQTT_Publish bmeHumidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bme280-humidity");

// Setup a feed called 'onoff' for subscribing to changes.
//Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");

/*************************** Sketch Code ************************************/

// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();
uint32_t readADC_Cal(int ADC_Raw);
void publish(float temp);
void show(uint32_t temp);
void printValues();

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SleftEEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SleftEEN_WIDTH, SleftEEN_HEIGHT, &Wire, OLED_RESET);

int raw = 0;
float temperature = 0.0;
float voltage = 0.0;
float prev_temperature = 0.0;

void setup() {
  Serial.begin(9600);

  delay(10);

   if(!display.begin(SSD1306_SWITCHCAPVCC, SleftEEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  delay(100); // Display Adafruit logo for a bit :)
  display.clearDisplay();

  //display.setRotation(2); // Uncomment to rotate display 180 degrees
  display.setTextSize(1);   // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);      // Use full 256 char 'Code Page 437' font

  display.setCursor(0, 0);  // 0,0 / 8 / 16 / 24 are the 21 char lines
  display.print("Booting...");
  display.display();

  // Connect to WiFi access point.
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

  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  bool status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  delay(2000);
}

void loop()
{
  raw = analogRead(LM_PIN);  
  voltage = readADC_Cal(raw);
  temperature = voltage / 10;
  Serial.printf("Temperature: %2.1f °C (LM35)\r\n", temperature);

  printValues(); // BME280

  if(temperature != prev_temperature) {
    show(temperature);
    publish(temperature);
    prev_temperature = temperature;
  }

  delay(30000);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void publish(float temp) 
{
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // Now we can publish stuff!
  Serial.print(F("\nSending temperature val "));
  Serial.printf("%2.1f", temperature);
  Serial.print("...");
  if (!temperatureFeed.publish(temp, 1)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  if (!bmeTemperatureFeed.publish(bme.readTemperature(), 1)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  if (!bmePressureFeed.publish(bme.readPressure() / 100.0F, 2)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  if (!bmeHumidityFeed.publish(bme.readHumidity(), 0)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // ping the server to keep the mqtt connection alive
  // NOT required if you are publishing once every KEEPALIVE seconds
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
}

void show(uint32_t temp)
{
    struct tm timeinfo;

    display.clearDisplay();
    display.setCursor(0, 0); // first row
    display.print("IP: " + WiFi.localIP().toString());
    display.setCursor(0, 12); // "second row"
    display.printf("Temperature: %2.1f C", temperature);
    display.setCursor(0, 24); // third row
    if(getLocalTime(&timeinfo)){
       display.printf("Last update: %02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    } else{ 
      Serial.println("Failed to obtain time");
    }
    display.display();
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}