/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

/************************* BME280 Setup *********************************/

#include <SPI.h>
#define BME_SCK 18 // SCL
#define BME_MISO 19 //SD0
#define BME_MOSI 23 //SDA
#define BME_CS 5 // CSB

/************************* LM35 Setup *********************************/
#define LM_PIN    36

/************************* SSD1306 DISPLAY Setup *********************************/
#define SleftEEN_WIDTH 128 // OLED display width, in pixels
#define SleftEEN_HEIGHT 32 // OLED display height, in pixels
