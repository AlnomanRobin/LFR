#include "Config.h"

static uint32_t btnLastScanMs = 0;
static bool     enterHolding  = false;
static bool     enterLongSent = false;
static uint32_t enterDownMs   = 0;

static bool     prevUpState   = HIGH;
static bool     prevDownState = HIGH;

void uiInit() {
  pinMode(PIN_BTN_UP,    INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN,  INPUT_PULLUP);
  pinMode(PIN_BTN_ENTER, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000L);
#if defined(WIRE_HAS_TIMEOUT)
  // Motor noise on SDA/SCL can lock the I2C bus forever; abort after 3 ms instead
  Wire.setWireTimeout(3000, true);
#endif

  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR) ||
              display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  if (oledReady) {
    showSplash();
  }
}

void showSplash() {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(16, 14);
  display.print(F("DIU SQUAD LFR"));
  display.setCursor(16, 32);
  display.print(F("6-CH IR Array"));
  display.setCursor(16, 48);
  display.print(F("Initializing..."));
  display.display();
  delay(600);
}

void showStatusScreen(const __FlashStringHelper* title, const __FlashStringHelper* line2) {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 14);
  display.print(title);
  display.setCursor(0, 34);
  display.print(line2);
  display.display();
}

void waitReleaseAll() {
  while (digitalRead(PIN_BTN_UP) == LOW ||
         digitalRead(PIN_BTN_DOWN) == LOW ||
         digitalRead(PIN_BTN_ENTER) == LOW) {
    delay(10);
  }
  prevUpState   = HIGH;
  prevDownState = HIGH;
  enterHolding  = false;
  enterLongSent = false;
  delay(30);
}

ButtonEvent readButtons() {
  uint32_t now = millis();
  if (now - btnLastScanMs < 25) {
    return BTN_NONE;
  }
  btnLastScanMs = now;

  bool curUp    = digitalRead(PIN_BTN_UP);
  bool curDown  = digitalRead(PIN_BTN_DOWN);
  bool curEnter = digitalRead(PIN_BTN_ENTER);

  // UP button (D2) falling edge
  if (prevUpState == HIGH && curUp == LOW) {
    prevUpState = LOW;
    return BTN_UP_PRESS;
  }
  prevUpState = curUp;

  // DOWN button (D10) falling edge
  if (prevDownState == HIGH && curDown == LOW) {
    prevDownState = LOW;
    return BTN_DOWN_PRESS;
  }
  prevDownState = curDown;

  // ENTER button (short press on release, long press while held)
  if (curEnter == LOW) {
    if (!enterHolding) {
      enterHolding  = true;
      enterLongSent = false;
      enterDownMs   = now;
    } else if (!enterLongSent && (now - enterDownMs >= 650)) {
      enterLongSent = true;
      return BTN_ENTER_LONG;
    }
  } else {
    if (enterHolding) {
      enterHolding = false;
      if (!enterLongSent) {
        return BTN_ENTER_PRESS;
      }
    }
  }

  return BTN_NONE;
}

