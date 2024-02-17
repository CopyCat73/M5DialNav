#include <TinyGPS++.h>
#include <TinyGPSPlus.h>
#include <M5_IMU_PRO.h>

#include "driver/temp_sensor.h"
#include "M5Dial.h"
M5Canvas img(&M5Dial.Display);

#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);

#include "Noto.h"
#include "smallFont.h"
#include "bigFont.h"
#include "secFont.h"

//gps
static const uint32_t GPSBaud = 9600;
#define RX_PIN 1
#define TX_PIN 2

#define BIM270_SENSOR_ADDR 0x68
#define BMP280_SENSOR_ADDR 0x76

BMI270::BMI270 bmi270;
Adafruit_BMP280 bmp(&Wire);
TinyGPSPlus gps;
HardwareSerial ss(2);

unsigned long time1;
unsigned long time2;

unsigned short grays[12];

int r = 116;
int sx = 120;
int sy = 120;

String cc[12] = { "N", "30", "60", "E", "120", "150", "S", "210", "240", "W", "300", "330" };

int start[12];
int startP[60];

int angle = 0;
int temperature = 0;
float pressure = 0.0;

int gpsSpeed = 0;
int gpsHeading = 0;
int satelliteCount = 0;
float lat, lng;

long oldEncoderPosition = -999;

#define CLOCK_R    240.0f / 2.0f // Clock face radius (float type)

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available()) gps.encode(ss.read());
  } while (millis() - start < ms);
}

void read_gps() {
  smartDelay(50);
  satelliteCount = gps.satellites.value();
  lat = gps.location.lat();
  lng = gps.location.lng();
  angle = int(round(gps.course.deg()));
  gpsSpeed = int(round(gps.speed.kmph()));
}

void read_environment() {
    temperature = int(round(bmp.readTemperature()));
    pressure = bmp.readPressure();
}

void read_compass() {
  if (bmi270.magneticFieldAvailable()) {
    int16_t x, y, z = 0;
    bmi270.readMagneticField(x, y, z);
    float xyHeading = atan2(x, y);
    float zxHeading = atan2(z, x);
    float heading = xyHeading;

    if (heading < 0) {
      heading += 2 * PI;
    }
    if (heading > 2 * PI) {
      heading -= 2 * PI;
    }
    float headingDegrees = heading * 180 / M_PI;
    angle = headingDegrees +40;
    float xyHeadingDegrees = xyHeading * 180 / M_PI;
    float zxHeadingDegrees = zxHeading * 180 / M_PI;

  }
}


void setup() {

  auto cfg = M5.config();
  M5Dial.begin(cfg, true, false);
  ss.begin(GPSBaud, SERIAL_8N1, RX_PIN, TX_PIN);
  M5.Ex_I2C.begin(I2C_NUM_0, 13, 15);

  unsigned status = bmp.begin(BMP280_SENSOR_ADDR);
  if (!status) {
    Serial.println(
      F("Could not find a valid BMP280 sensor, check wiring or "
        "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    while (1) delay(10);
  }

  bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);

  M5Dial.Rtc.setDateTime({ { 2023, 10, 25 }, { 15, 56, 56 } });
  sprite.createSprite(240, 240);
  sprite.setSwapBytes(true);
  sprite.setTextDatum(4);


  int co = 210;
  for (int i = 0; i < 12; i++) {
    grays[i] = tft.color565(co, co, co);
    co = co - 20;
  }

  time1 = millis();
  time2 = millis();
}

void encoder_read() {
  M5Dial.update();
  int currentAngle = angle;
  long newEncoderPosition = M5Dial.Encoder.read();
  if (newEncoderPosition > oldEncoderPosition) {
    //M5Dial.Speaker.tone(8000, 20);
    oldEncoderPosition = newEncoderPosition;
    /* testing screen rotation with encoder when no gps present
    currentAngle = currentAngle + 10;
    if (currentAngle >= 360) {
      currentAngle = 0;
    }
    angle = currentAngle +10;
    */
  }
}

int inc = 0;

void loop() {

  auto dt = M5Dial.Rtc.getDateTime();

  if (millis() - time1 >= 100) {
    read_gps();
    read_environment();
    //read_compass();
    time1 = millis();
  }

  if (millis() - time2 >= 1000) {
    //compass();
    time2 = millis();
  }

  encoder_read();

  sprite.fillSprite(TFT_BLACK);
  sprite.loadFont(smallFont);
  sprite.setTextColor(grays[4], TFT_BLACK);
  sprite.drawString(String(temperature)+char(176), sx -60, sy +10);
  sprite.unloadFont();

  sprite.loadFont(smallFont);
  sprite.setTextColor(grays[4], TFT_BLACK);
  sprite.drawNumber(satelliteCount, sx+ 60, sy +10);
  sprite.unloadFont();

  sprite.loadFont(secFont);
  sprite.setTextColor(grays[1], TFT_BLACK);
  sprite.drawString(String(angle), sx, sy - 42);
  sprite.unloadFont();

  sprite.loadFont(bigFont);
  sprite.setTextColor(grays[0], TFT_BLACK);
  sprite.drawString(String(gpsSpeed), sx, sy+10);
  sprite.unloadFont();

  sprite.loadFont(smallFont);
  sprite.setTextColor(grays[0], TFT_BLACK);
  sprite.drawString("km/h", sx, sy + 42);
  sprite.unloadFont();

  sprite.loadFont(Noto);
  sprite.setTextColor(TFT_ORANGE, TFT_BLACK);
  sprite.drawString("M5DIAL", 120, 190);
  
  
  float xp = 0.0, yp = 0.0, xps = 0.0, yps = 0.0, xpe = 0.0, ype = 0.0; 
  int h;

  for (h = 0; h < 60; h++) {
    
    int offset = (h*6)-angle;
    if (offset <0) {
      offset = offset + 360;
    }
    getCoord(CLOCK_R, CLOCK_R, &xp, &yp, CLOCK_R -5, offset );
    sprite.fillSmoothCircle(xp, yp, 1, grays[4], TFT_BLACK);
  }

  sprite.setTextColor(grays[3],TFT_BLACK);
  for (h = 0; h < 12; h++) {
    int offset = (h*30)-angle;
    if (offset <0) {
      offset = offset + 360;
    }
    getCoord(CLOCK_R, CLOCK_R, &xps, &yps, CLOCK_R -5, offset );
    getCoord(CLOCK_R, CLOCK_R, &xpe, &ype, CLOCK_R -10, offset );
    sprite.drawWedgeLine(xps, yps, xpe, ype, 2, 2, grays[3], TFT_BLACK);
    getCoord(CLOCK_R, CLOCK_R, &xp, &yp, CLOCK_R -25, offset );
    sprite.drawString(cc[h], xp, 2 + yp);
  }
    sprite.drawTriangle(sx-5, sy-70, sx +5, sy-70 ,sx, sy -82,  TFT_ORANGE);
    M5Dial.Display.pushImage(0, 0, 240, 240, (uint16_t*)sprite.getPointer());
    sprite.unloadFont();
  
}

// =========================================================================
// Get coordinates of end of a line, pivot at x,y, length r, angle a
// =========================================================================
// Coordinates are returned to caller via the xp and yp pointers
#define DEG2RAD 0.0174532925
void getCoord(int16_t x, int16_t y, float *xp, float *yp, int16_t r, float a)
{
  float sx1 = cos( (a - 90) * DEG2RAD);
  float sy1 = sin( (a - 90) * DEG2RAD);
  *xp =  sx1 * r + x;
  *yp =  sy1 * r + y;
}
