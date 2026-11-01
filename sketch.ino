#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// =====================================================
// PIN CONFIGURATION
// =====================================================
#define I2C_SDA     8
#define I2C_SCL     9

#define DHT_PIN     4

#define LDR_PIN     1
#define BUZZER_PIN  18

#define TFT_CS      10
#define TFT_DC      13
#define TFT_MOSI    11
#define TFT_SCK     12

// =====================================================
// SETTINGS
// =====================================================
#define SENSOR_INTERVAL_MS  2500   // DHT22 needs more than 2 s between readings
#define SERIAL_INTERVAL_MS  5000
#define LDR_SAMPLES         8
#define LDR_INVERT          false  // set true if your LDR module gives HIGH value in the dark
#define DHT_MAX_FAILS       3      // consecutive failures before showing "--"

// Rain thresholds
#define HUM_HIGH     80.0f
#define HUM_MID      70.0f
#define PRES_LOW     1010.0f
#define PRES_MID     1015.0f
#define LIGHT_LOW    500
#define LIGHT_MID    800

// Screen layout (320x240, rotation 1, text size 2 = 12x16 px per char)
#define ROW_X        10
#define ROW_Y0       36
#define ROW_STEP     26
#define LINE_CHARS   26            // 26 * 12 px = 312 px

// =====================================================
// OBJECTS
// =====================================================
Adafruit_BMP085   bmp;
Adafruit_ILI9341  tft(TFT_CS, TFT_DC);

// =====================================================
// DATA
// =====================================================
enum Prediction { NO_RAIN, RAIN_LIKELY, RAINING };

float temperature = NAN;
float humidity    = NAN;
float pressure    = NAN;
int   light       = 0;

int        rainScore      = 0;
int        rainConditions = 0;
Prediction prediction     = NO_RAIN;

bool          buzzerOn      = false;
uint8_t       dhtFailCount  = 0;
unsigned long lastSensorMs  = 0;
unsigned long lastSerialMs  = 0;

// =====================================================
// TFT HELPERS
// =====================================================

// Prints one padded line with a background colour, so old text is
// overwritten without clearing the screen (no flicker).
void drawLine(int y, uint16_t color, const char *text, uint8_t size = 2) {
  tft.setTextSize(size);
  tft.setTextColor(color, ILI9341_BLACK);
  tft.setCursor(ROW_X, y);

  char buf[LINE_CHARS + 1];
  snprintf(buf, sizeof(buf), "%-*s", LINE_CHARS, text);
  tft.print(buf);
}

void drawTitle() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK);
  tft.setCursor(55, 8);
  tft.print("WEATHER STATION");
}

const char *predictionText(Prediction p) {
  switch (p) {
    case RAINING:     return "RAINING";
    case RAIN_LIKELY: return "RAIN LIKELY";
    default:          return "NO RAIN";
  }
}

uint16_t predictionColor(Prediction p) {
  switch (p) {
    case RAINING:     return ILI9341_RED;
    case RAIN_LIKELY: return ILI9341_YELLOW;
    default:          return ILI9341_GREEN;
  }
}

// =====================================================
// SENSORS
// =====================================================

// Counts how long the data line stays at `level`. The DHT22 library's 90 us
// timeout never completes in this simulator, so bits are judged by comparing
// the low and high pulse lengths instead (short high = 0, long high = 1).
uint32_t dhtPulse(bool level) {
  uint32_t count = 0;
  while (digitalRead(DHT_PIN) == level) {
    if (++count >= 100000) return UINT32_MAX;
  }
  return count;
}

// One DHT22 sample. Returns false on timeout or a bad checksum.
bool readDht22(float &tempC, float &hum) {
  uint8_t data[5] = {0, 0, 0, 0, 0};
  uint32_t cycles[80];

  pinMode(DHT_PIN, INPUT_PULLUP);
  delay(1);

  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, LOW);
  delayMicroseconds(1100);             // start signal, datasheet says >= 1 ms

  pinMode(DHT_PIN, INPUT_PULLUP);
  delayMicroseconds(40);               // sensor then pulls the line low

  if (dhtPulse(LOW) == UINT32_MAX)  return false;
  if (dhtPulse(HIGH) == UINT32_MAX) return false;

  for (int i = 0; i < 80; i += 2) {
    cycles[i]     = dhtPulse(LOW);
    cycles[i + 1] = dhtPulse(HIGH);
  }

  for (int i = 0; i < 40; i++) {
    uint32_t lowCycles  = cycles[2 * i];
    uint32_t highCycles = cycles[2 * i + 1];
    if (lowCycles == UINT32_MAX || highCycles == UINT32_MAX) return false;
    data[i / 8] <<= 1;
    if (highCycles > lowCycles) data[i / 8] |= 1;
  }

  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) return false;

  hum   = ((uint16_t)data[0] << 8 | data[1]) * 0.1f;
  int16_t rawT = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];
  if (data[2] & 0x80) rawT = -rawT;
  tempC = rawT * 0.1f;
  return true;
}

void readSensors() {
  // BMP180 supplies pressure. Its temperature is only a fallback for the
  // DHT22, so the on-screen Temp follows the DHT22 slider.
  float bmpTemp = bmp.readTemperature();
  pressure      = bmp.readPressure() / 100.0f;

  float dhtTemp, dhtHum;
  if (readDht22(dhtTemp, dhtHum)) {
    temperature  = dhtTemp;
    humidity     = dhtHum;
    dhtFailCount = 0;
  } else {
    temperature = bmpTemp;
    if (dhtFailCount < 255) dhtFailCount++;
    if (dhtFailCount >= DHT_MAX_FAILS) humidity = NAN;
    Serial.println("WARNING: DHT22 read failed");
  }

  // LDR (averaged to reduce noise)
  long sum = 0;
  for (int i = 0; i < LDR_SAMPLES; i++) sum += analogRead(LDR_PIN);
  int raw = sum / LDR_SAMPLES;

  light = map(raw, 0, 4095, 0, 1000);
  if (LDR_INVERT) light = 1000 - light;
  light = constrain(light, 0, 1000);
}

// =====================================================
// RAIN PREDICTION
// =====================================================
void setBuzzer(bool on) {
  if (on == buzzerOn) return;          // only touch the pin when state changes
  buzzerOn = on;
  if (on) tone(BUZZER_PIN, 1000);
  else    noTone(BUZZER_PIN);
}

void calculatePrediction() {
  rainScore      = 0;
  rainConditions = 0;

  // Humidity (skipped if DHT22 has no valid reading)
  if (!isnan(humidity)) {
    if (humidity >= HUM_HIGH)     { rainScore += 40; rainConditions++; }
    else if (humidity >= HUM_MID) { rainScore += 20; }
  }

  // Pressure
  if (!isnan(pressure)) {
    if (pressure <= PRES_LOW)      { rainScore += 35; rainConditions++; }
    else if (pressure <= PRES_MID) { rainScore += 15; }
  }

  // Light
  if (light <= LIGHT_LOW)      { rainScore += 25; rainConditions++; }
  else if (light <= LIGHT_MID) { rainScore += 10; }

  rainScore = min(rainScore, 100);

  if (rainConditions >= 2)                           prediction = RAINING;
  else if (rainConditions == 1 || rainScore >= 40)   prediction = RAIN_LIKELY;
  else                                               prediction = NO_RAIN;

  setBuzzer(prediction == RAINING);
}

// =====================================================
// DISPLAY
// =====================================================
void displayData() {
  char buf[LINE_CHARS + 1];
  int y = ROW_Y0;

  snprintf(buf, sizeof(buf), "Temp: %.1f C", temperature);
  drawLine(y, ILI9341_YELLOW, buf);
  y += ROW_STEP;

  if (isnan(humidity)) snprintf(buf, sizeof(buf), "Humidity: -- %%");
  else                 snprintf(buf, sizeof(buf), "Humidity: %.1f %%", humidity);
  drawLine(y, ILI9341_CYAN, buf);
  y += ROW_STEP;

  snprintf(buf, sizeof(buf), "Pressure: %.1f hPa", pressure);
  drawLine(y, ILI9341_WHITE, buf);
  y += ROW_STEP;

  snprintf(buf, sizeof(buf), "Light: %d lux", light);
  drawLine(y, ILI9341_ORANGE, buf);
  y += ROW_STEP;

  snprintf(buf, sizeof(buf), "Rain Score: %d/100", rainScore);
  drawLine(y, ILI9341_MAGENTA, buf);
  y += ROW_STEP;

  snprintf(buf, sizeof(buf), "Conditions: %d/3", rainConditions);
  drawLine(y, ILI9341_WHITE, buf);

  // Prediction on one line so it always fits on screen
  snprintf(buf, sizeof(buf), "Prediction: %s", predictionText(prediction));
  drawLine(196, predictionColor(prediction), buf);

  // Buzzer status (small text)
  drawLine(224, buzzerOn ? ILI9341_RED : ILI9341_GREEN,
           buzzerOn ? "BUZZER: ON" : "BUZZER: OFF", 1);
}

// =====================================================
// SERIAL
// =====================================================
void printSerialData() {
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("       WEATHER STATION"));
  Serial.println(F("================================"));
  Serial.printf("Temperature : %.1f C\n", temperature);

  if (isnan(humidity)) Serial.println(F("Humidity    : -- (sensor error)"));
  else                 Serial.printf("Humidity    : %.1f %%\n", humidity);

  Serial.printf("Pressure    : %.1f hPa\n", pressure);
  Serial.printf("Light       : %d lux\n", light);
  Serial.println(F("--------------------------------"));
  Serial.printf("Rain Score  : %d/100\n", rainScore);
  Serial.printf("Conditions  : %d/3\n", rainConditions);
  Serial.printf("Prediction  : %s\n", predictionText(prediction));
  Serial.printf("Buzzer      : %s\n", buzzerOn ? "ON" : "OFF");
  Serial.println(F("================================"));
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  analogReadResolution(12);
  pinMode(LDR_PIN, INPUT);

  // TFT first, so errors can be shown on screen
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(55, 30);
  tft.print("WEATHER STATION");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(70, 80);
  tft.print("Starting...");

  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(DHT_PIN, INPUT_PULLUP);          // DHT22 data line idles high

  if (!bmp.begin()) {
    Serial.println(F("ERROR: BMP180 NOT FOUND!"));
    tft.setTextColor(ILI9341_RED);
    tft.setCursor(30, 130);
    tft.print("BMP180 NOT FOUND!");
    while (true) delay(1000);
  }
  Serial.println(F("BMP180 OK"));

  delay(1500);
  drawTitle();

  // First DHT22 response often times out. Wait a full sample gap and try once more.
  delay(2000);
  readSensors();
  if (isnan(humidity)) {
    delay(SENSOR_INTERVAL_MS);
    readSensors();
  }
  calculatePrediction();
  displayData();
  printSerialData();

  lastSensorMs = lastSerialMs = millis();
}

// =====================================================
// LOOP (non-blocking)
// =====================================================
void loop() {
  unsigned long now = millis();

  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;
    readSensors();
    calculatePrediction();
    displayData();
  }

  if (now - lastSerialMs >= SERIAL_INTERVAL_MS) {
    lastSerialMs = now;
    printSerialData();
  }
}
