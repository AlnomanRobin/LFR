#include <Wire.h>
#include <EEPROM.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

SSD1306AsciiWire oled;

/**************** PINS ****************/

// Control Buttons
#define BTN_UP     2    // UP / NEXT – D2
#define BTN_DOWN   10   // DOWN / PREV – D10
#define BTN_OK     11   // OK / ENTER – D11

#define LED_PIN    13

// TB6612FNG Motor Driver
#define PWMA       3    // PWMA – D3
#define AIN2       4    // AIN2 – D4
#define AIN1       5    // AIN1 – D5
#define STBY       6    // STBY – D6
#define BIN1       7    // BIN1 – D7
#define BIN2       8    // BIN2 – D8
#define PWMB       9    // PWMB – D9

// Motor Direction & Speed Aliases
#define LPWM       PWMA // Left Motor PWM (D3)
#define LFWD       AIN1 // Left Motor Forward (D5) - swap with AIN2 if reversed
#define LBWD       AIN2 // Left Motor Backward (D4)

#define RPWM       PWMB // Right Motor PWM (D9)
#define RFWD       BIN1 // Right Motor Forward (D7) - swap with BIN2 if reversed
#define RBWD       BIN2 // Right Motor Backward (D8)

/************* SENSOR PINS ************/

// IR Sensor Array (S1 to S6: Left to Right)
byte sensorPin[6] =
{
  A7, // S1 – A7
  A6, // S2 – A6
  A3, // S3 – A3
  A2, // S4 – A2
  A1, // S5 – A1
  A0  // S6 – A0
};

/************* VARIABLES **************/

int sensorValue[6];
int sensorMin[6];
int sensorMax[6];
int sensorMid[6];

byte sensorDigital[6];

int menuIndex = 1;

bool inverseMode = false;
bool autoInverseEnabled = true;

/************* SPEED & PID ************/

float Kp = 25.0;
float Ki = 0.0;
float Kd = 22.0;

float error = 0;
float lastError = 0;
float integral = 0;

int baseSpeed = 210;
int maxSpeed  = 255;
int gapSpeed  = 190;
int turnSpeed = 165;

// Track timing variables for Gap, Inverse & Finish
unsigned long runStartTime   = 0;
unsigned long gapStartTime   = 0;
const unsigned long GAP_TIMEOUT = 320; // ms to shoot across dashed line
unsigned long floodStartTime = 0;
unsigned long lastFlipTime   = 0;

/****************************************/

void setup()
{
  // Fast ADC prescaler (16us conversion -> 6x faster than default Arduino)
  ADCSRA = (ADCSRA & 0xF8) | 0x05;

  Wire.begin();

  oled.begin(&Adafruit128x64, 0x3C);
  oled.setFont(Adafruit5x7);
  oled.clear();

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Enable TB6612FNG Motor Driver
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(LFWD, OUTPUT);
  pinMode(LBWD, OUTPUT);

  pinMode(RFWD, OUTPUT);
  pinMode(RBWD, OUTPUT);

  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);

  stopMotor();

  splash();

  loadCalibration();
  loadPIDSettings();
}

void loop()
{
  menuSystem();
}
