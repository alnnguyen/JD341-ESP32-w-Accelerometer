#include <Arduino.h>
// Vibration G-Force Meter — ESP32 + MPU6050 + GME12864 (SSD1306 128x64 I2C
// OLED) ER3: display g-force in g, 0.1 g resolution, update >= 1x per second
//
// Wiring (shared I2C bus):
//   ESP32 3V3  -> MPU6050 VCC, OLED VCC
//   ESP32 GND  -> MPU6050 GND, OLED GND
//   ESP32 GPIO21 (SDA) -> MPU6050 SDA, OLED SDA
//   ESP32 GPIO22 (SCL) -> MPU6050 SCL, OLED SCL
//   MPU6050 AD0 -> GND (or leave floating) => address 0x68
//   OLED address is usually 0x3C
//
// Libraries (Arduino Library Manager):
//   Adafruit MPU6050, Adafruit Unified Sensor, Adafruit SSD1306, Adafruit GFX

#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define OLED_ADDR 0x3C
#define SCREEN_W 128
#define SCREEN_H 64

const float G = 9.80665f;            // m/s^2 per g
const uint32_t SAMPLE_PERIOD_MS = 5; // ~200 Hz sampling
const uint32_t DISPLAY_PERIOD_MS =
    250;                              // 4 display updates/sec (ER3 needs >= 1)
const uint32_t PEAK_WINDOW_MS = 1000; // peak held over last 1 s

Adafruit_MPU6050 mpu;
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

float gx = 0, gy = 0, gz = 0, gMag = 0;
float peakVib = 0, peakVibShown = 0;
uint32_t lastSample = 0, lastDisplay = 0, peakStart = 0;

void haltWithMessage(const char *msg) {
  Serial.println(msg);
  while (true)
    delay(1000);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // fast-mode I2C

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    haltWithMessage("OLED not found - check wiring/address (try 0x3D)");
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("G-Force Meter");
  display.println("Starting...");
  display.display();

  if (!mpu.begin(0x68, &Wire)) {
    display.println("MPU6050 not found!");
    display.display();
    haltWithMessage("MPU6050 not found - check wiring (AD0 low = 0x68)");
  }
  mpu.setAccelerometerRange(
      MPU6050_RANGE_8_G); // +/-8 g; use 16_G for harsh vibration
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // on-chip low-pass filter

  peakStart = millis();
}

void sampleSensor() {
  sensors_event_t a, gyro, temp;
  mpu.getEvent(&a, &gyro, &temp);

  gx = a.acceleration.x / G;
  gy = a.acceleration.y / G;
  gz = a.acceleration.z / G;
  gMag = sqrtf(gx * gx + gy * gy + gz * gz); // total g (reads ~1.0 at rest)

  float vib = fabsf(gMag - 1.0f); // dynamic g, gravity removed
  if (vib > peakVib)
    peakVib = vib;

  if (millis() - peakStart >= PEAK_WINDOW_MS) {
    peakVibShown = peakVib;
    peakVib = 0;
    peakStart = millis();
  }
}

void updateDisplay() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Total g:");

  display.setTextSize(3);
  display.setCursor(0, 12);
  display.print(gMag, 1); // 0.1 g resolution
  display.setTextSize(2);
  display.print(" g");

  display.setTextSize(1);
  display.setCursor(0, 40);
  display.print("Vib pk(1s): ");
  display.print(peakVibShown, 1);
  display.print(" g");

  display.setCursor(0, 54);
  display.print("X");
  display.print(gx, 1);
  display.print(" Y");
  display.print(gy, 1);
  display.print(" Z");
  display.print(gz, 1);

  display.display();

  Serial.printf("x=%.2f y=%.2f z=%.2f |g|=%.2f vibPk=%.2f\n", gx, gy, gz, gMag,
                peakVibShown);
}

void loop() {
  uint32_t now = millis();
  if (now - lastSample >= SAMPLE_PERIOD_MS) {
    lastSample = now;
    sampleSensor();
  }
  if (now - lastDisplay >= DISPLAY_PERIOD_MS) {
    lastDisplay = now;
    updateDisplay();
  }
}
