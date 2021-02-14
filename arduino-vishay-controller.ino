#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <timer.h>
#include "Sensor.h"

#define PIN        10
#define PS_MIN_HYST 680
#define PS_MAX_HYST 700

#define DEVICE_ADDR                                 0x51
#define CMD_DEVICE_ID                               0x0E
#define CMD_PS_THDL                                 0x06
#define CMD_PS_THDH                                 0x07
#define CMD_PS1_DATA                                0x08
#define CMD_ALS_DATA                                0x0B
#define CMD_WHITE_DATA                              0x0C
#define CMD_ALS_CONF1_2                             0x00
#define CMD_PS_CONF1_2                              0x03
#define CMD_PS_CONF3_MS                             0x04

#define CMD_INT_FLAG                                0x0D

#define PS_THDL_M                                   0x03  // THDL = 800
#define PS_THDL_L                                   0x20
#define PS_THDH_M                                   0x03  // THDH = 1000
#define PS_THDH_L                                   0xE8

#define ALS_CONF1                                   0x12  // 50 ms intergration time, ALS enabled, dynamic range x2
#define ALS_CONF2                                   0x00  // sensitivity x2, White enabled
#define PS_CONF1                                    0x3E  // PS enabled, PS interrupt persistence 4, 8T integration time
#define PS_CONF2                                    0x4B  // PS 16-bit output, PS interrupt on closing/away, gesture enabled
#define PS_CONF3                                    0x0D  // PS Sunlight Cancellation, active force mode, force trigger
#define PS_MS                                       0x07  // 200 mA LED_I current

#define WRITE_DATA(CMD, LSB, MSB) {\
  Wire.beginTransmission(DEVICE_ADDR);\
  Wire.write(CMD);\
  Wire.write(LSB);\
  Wire.write(MSB);\
  Wire.endTransmission(true);\
}

#define READ_DATA(DATA, CMD) {\
  Wire.beginTransmission(DEVICE_ADDR);\
  Wire.write(CMD);\
  Wire.endTransmission(false);\
  Wire.requestFrom(DEVICE_ADDR, 2, true);\
  DATA[0] = Wire.read();\
  DATA[1] = Wire.read();\
}

// LED Macros
#define NUMPIXELS 15 // Popular NeoPixel ring size
#define LED_R_VAL 255
#define LED_G_VAL 244
#define LED_B_VAL 150

/*
 * Global Variable
 */
auto timer = timer_create_default();  // Timer Helper Class
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
volatile char readData[2] = {0};
volatile uint32_t toggleCount = 0;
volatile bool ledToggle = LOW;
volatile uint16_t ps1_data, als_data;
double intensity;

uint16_t proximityTable[DIST_LOOKUP_LEN] = {
  65535, 18000, 4000, 2000, 1275, 1150, 920, 810, 765, 740, 720, 710, 700, 690, 680, 670
};
Sensor sensor;

/*
 * Function Prototypes
 */
void sensorSetup(void);
bool sensorQuery(void *);
bool serialQuery(void *);
bool ledUpdate(void *);
void sampleSensor(void);
void setWhiteLED(double intensity);

void setup() {
  Wire.begin();
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, ledToggle);

  sensorSetup();
  
  pixels.begin();
  setWhiteLED(0.01);

  // Set up timer tasks
  timer.every(10, sensorQuery);
  timer.every(100, serialQuery);
  timer.every(50, ledUpdate);
}

void loop() {
  // Tick the timer forward to trigger assigned task callbacks
  timer.tick();
}

void sensorSetup(void) {
  // Check Device ID - should be 0x80, 0x00
  READ_DATA(readData, CMD_DEVICE_ID);

  // ALS Config
  WRITE_DATA(CMD_ALS_CONF1_2, ALS_CONF1, ALS_CONF2);

  // Proximity Sensor Config
  WRITE_DATA(CMD_PS_CONF1_2, PS_CONF1, PS_CONF2);
  WRITE_DATA(CMD_PS_CONF3_MS, PS_CONF3, PS_MS);

  // PS INT Settings
  WRITE_DATA(CMD_PS_THDL, PS_THDL_L, PS_THDL_M);
  WRITE_DATA(CMD_PS_THDH, PS_THDH_L, PS_THDH_M);
  
  // Clear any interrupt flags
  READ_DATA(readData, CMD_INT_FLAG);

  // Set up sensor struct
  Init_Sensor(&sensor, 0, PS_MIN_HYST, PS_MAX_HYST, proximityTable);
}

bool sensorQuery(void *) {
  sampleSensor();
  digitalWrite(LED_BUILTIN, ledToggle);
  return true;
}

void sampleSensor(void) {
  WRITE_DATA(CMD_PS_CONF3_MS, PS_CONF3, PS_MS);  
  READ_DATA(readData, CMD_PS1_DATA);
  ps1_data = *((uint16_t*)readData);
  READ_DATA(readData, CMD_ALS_DATA);
  als_data = *((uint16_t*)readData);

  Update_Sensor(&sensor, ps1_data, als_data);
  
  if (!ledToggle && sensor.inProximity) {
    toggleCount += 1;
  }

  ledToggle = sensor.inProximity;

  // Update intensity if inProximity
  if (ledToggle) {
    // First, normalize in distance range, then use tan(x) as activation function, saturate at 1
    intensity = min(sensor.estimatedDistance, 25);
    intensity = max(sensor.estimatedDistance, 5);
    
    intensity = (intensity - 5)/20;
    intensity = tan(intensity * 0.8);
    
    intensity = max(intensity, 0.008);
    intensity = min(intensity, 1);
  }
}

void setWhiteLED(double intensity) {
  int r = (int) (LED_R_VAL * intensity);
  int g = (int) (LED_G_VAL * intensity);
  int b = (int) (LED_B_VAL * intensity);

  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();   // Send the updated pixel colors to the hardware.
}

// Timer callback functions

bool serialQuery(void *) {
  // Update latest data every 10 ms
  Serial.print(ps1_data);
  Serial.print(",");
  Serial.print(als_data);
  Serial.print("\t\t");
  Serial.print(toggleCount);
  Serial.print(",");
  Serial.print(sensor.psMean);
  Serial.print(",");
  Serial.print(sensor.alsMean);
  Serial.print(",");
  Serial.print(sensor.estimatedDistance);
  Serial.print(",");
  Serial.print(sensor.inProximity);
  Serial.println("");
  return true;
}

bool ledUpdate(void *) {
  if ((toggleCount % 2) == 1) {
    setWhiteLED(intensity);
  } else {
    setWhiteLED(0);
  }

  return true;
}
