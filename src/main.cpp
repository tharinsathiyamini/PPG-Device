#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pin map ----------
#define CS_PIN   5
#define SCK_PIN  18
#define MOSI_PIN 23
#define MISO_PIN 19

#define OLED_SDA 21
#define OLED_SCL 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define MOTION_PIN 4 // straight from USER PPG's MOTION_OUT - stands in for a
                      // real accelerometer's motion-detected flag

// ---------- AFE register map (must match chips/afe/chip.c) ----------
#define REG_RED_MSB     0x00
#define REG_IR_MSB      0x02
#define REG_STATUS      0x04
#define REG_LED_CURRENT 0x05
#define REG_GAIN        0x06
#define REG_SAMPLE_RATE 0x07

// ---------- Network config ----------
// "Wokwi-GUEST" is the open access point provided by the Wokwi simulator
// that has real (proxied) internet access - no password required.
const char *WIFI_SSID = "Wokwi-GUEST";
const char *WIFI_PASS = "";

const char *MQTT_BROKER = "broker.hivemq.com";
const int   MQTT_PORT   = 1883;
const char *MQTT_TOPIC  = "bm2210/ppg-demo/alarm"; // change to something unique for your report

// ---------- Clinical alarm thresholds ----------
#define HR_LOW_BPM    50
#define HR_HIGH_BPM   120
#define SPO2_LOW_PCT  92.0f

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- AFE SPI helpers ----------
static void afeBegin() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
}
static void afeEnd() {
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}
static uint16_t afeReadReg16(uint8_t addr) {
  afeBegin();
  SPI.transfer(addr & 0x7F);
  uint8_t msb = SPI.transfer(0x00);
  uint8_t lsb = SPI.transfer(0x00);
  afeEnd();
  return ((uint16_t)msb << 8) | lsb;
}
static uint8_t afeReadReg8(uint8_t addr) {
  afeBegin();
  SPI.transfer(addr & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  afeEnd();
  return v;
}
static void afeWriteReg8(uint8_t addr, uint8_t val) {
  afeBegin();
  SPI.transfer(addr | 0x80);
  SPI.transfer(val);
  afeEnd();
}

// ---------- DSP state ----------
float redDcEma = 1.4f, irDcEma = 1.8f;
const float DC_ALPHA = 0.01f; // low-pass EMA coefficient (~cutoff << 0.5 Hz cardiac band)

float redAcMin = 0, redAcMax = 0, irAcMin = 0, irAcMax = 0;
float irAcPrev = 0;
int   irSlopePrev = 0;

unsigned long lastBeatMs = 0;
const unsigned long REFRACTORY_MS = 300; // per the design hints

float hrBpm = 0;
float spo2Pct = 98.0f;
bool  alarmActive = false;

// ---------- Motion-artifact rejection ----------
bool ARTIFACT_REJECTION_ENABLED = true;
bool motionActive = false;
unsigned long motionHoldUntilMs = 0;
const unsigned long MOTION_HANGOVER_MS = 800; // keep rejecting briefly after
                                               // motion stops, so filters can
                                               // re-settle before we trust them again

// scrolling waveform buffer for the OLED
int16_t waveBuf[SCREEN_WIDTH];
int waveIdx = 0;
int sampleDecim = 0; // decimate ~4 AFE cycles per OLED column

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println(" connected!");
  configTime(0, 0, "pool.ntp.org"); // UTC; adjust offsets if you want local time
}

void connectMQTT() {
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  while (!mqtt.connected()) {
    String clientId = "bm2210-esp32-" + String(random(0xffff), HEX);
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("MQTT connected");
    } else {
      delay(1000);
    }
  }
}

String isoTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 2000)) {
    return "1970-01-01T00:00:00Z";
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}

void publishAlarm(const char *reason) {
  if (!mqtt.connected()) connectMQTT();
  char payload[220];
  snprintf(payload, sizeof(payload),
    "{\"hr\":%.1f,\"spo2\":%.1f,\"alarm\":%s,\"reason\":\"%s\",\"timestamp\":\"%s\"}",
    hrBpm, spo2Pct, alarmActive ? "true" : "false", reason, isoTimestamp().c_str());
  mqtt.publish(MQTT_TOPIC, payload);
  Serial.println(payload);
}

void updateOled() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("HR:%3d bpm  SpO2:%3d%%", (int)hrBpm, (int)spo2Pct);
  if (motionActive) {
    display.setCursor(80, 0);
    display.print("MOT");
  } else if (alarmActive) {
    display.setCursor(90, 0);
    display.print("ALM");
  }

  // scrolling waveform in the lower 40 px of the screen
  int baseY = 24, plotH = 36;
  for (int x = 0; x < SCREEN_WIDTH - 1; x++) {
    int idxA = (waveIdx + x) % SCREEN_WIDTH;
    int idxB = (waveIdx + x + 1) % SCREEN_WIDTH;
    display.drawLine(x, baseY + plotH - waveBuf[idxA],
                      x + 1, baseY + plotH - waveBuf[idxB], SSD1306_WHITE);
  }
  display.display();
}

void processSample(uint16_t redCode, uint16_t irCode) {
  float redV = (redCode / 65535.0f) * 3.3f;
  float irV  = (irCode  / 65535.0f) * 3.3f;

  redDcEma += DC_ALPHA * (redV - redDcEma);
  irDcEma  += DC_ALPHA * (irV  - irDcEma);

  float redAc = redV - redDcEma;
  float irAc  = irV  - irDcEma;

  if (redAc > redAcMax) redAcMax = redAc;
  if (redAc < redAcMin) redAcMin = redAc;
  if (irAc  > irAcMax)  irAcMax  = irAc;
  if (irAc  < irAcMin)  irAcMin  = irAc;

  // --- push a point into the scrolling waveform buffer ---
  sampleDecim++;
  if (sampleDecim >= 4) {
    sampleDecim = 0;
    int16_t y = (int16_t)constrain(irAc * 4000.0f, -18, 18) + 18;
    waveBuf[waveIdx] = y;
    waveIdx = (waveIdx + 1) % SCREEN_WIDTH;
  }

  // --- peak detector on IR (foot-to-foot / systolic peak), refractory 300 ms ---
  int slope = (irAc > irAcPrev) ? 1 : -1;
  unsigned long now = millis();
  float dynamicThresh = 0.25f * (irAcMax - irAcMin);
  if (irSlopePrev > 0 && slope < 0 &&
      irAc > dynamicThresh &&
      (now - lastBeatMs) > REFRACTORY_MS) {

    unsigned long interval = now - lastBeatMs;
    bool suppressUpdate = ARTIFACT_REJECTION_ENABLED && motionActive;

    if (lastBeatMs != 0 && interval > 300 && interval < 2000 && !suppressUpdate) {
      float newHr = 60000.0f / interval;
      hrBpm = (hrBpm == 0) ? newHr : (0.7f * hrBpm + 0.3f * newHr); // light smoothing
    }
    lastBeatMs = now;

    // Ratio-of-ratios SpO2 estimate, using the AC swing accumulated over
    // the cycle that just completed.
    float ppRed = redAcMax - redAcMin;
    float ppIr  = irAcMax - irAcMin;
    if (redDcEma > 0.05f && irDcEma > 0.05f && ppIr > 0.0005f && !suppressUpdate) {
      float acdcRed = ppRed / redDcEma;
      float acdcIr  = ppIr  / irDcEma;
      float R = acdcRed / acdcIr;
      float newSpo2 = 110.0f - 25.0f * R; // empirical calibration - curve fit,
                                           // not first-principles (state this
                                           // limitation in your report)
      newSpo2 = constrain(newSpo2, 70.0f, 100.0f);
      spo2Pct = 0.7f * spo2Pct + 0.3f * newSpo2;
    }

    // reset the per-cycle envelope trackers for the next beat
    redAcMax = redAcMin = redAc;
    irAcMax = irAcMin = irAc;
  }
  irSlopePrev = slope;
  irAcPrev = irAc;

  // --- alarm zone check (suppressed while a motion artifact is present) ---
  bool prevAlarm = alarmActive;
  bool hrBad = (hrBpm > 0) && (hrBpm < HR_LOW_BPM || hrBpm > HR_HIGH_BPM);
  bool spo2Bad = spo2Pct < SPO2_LOW_PCT;

  if (!(ARTIFACT_REJECTION_ENABLED && motionActive)) {
    alarmActive = hrBad || spo2Bad;
    if (alarmActive && !prevAlarm) {
      const char *reason = hrBad && spo2Bad ? "hr_and_spo2" : (hrBad ? "hr_out_of_range" : "desaturation");
      publishAlarm(reason);
    } else if (!alarmActive && prevAlarm) {
      publishAlarm("cleared");
    }
  }

  // CSV-style log for before/after plots (Arduino/PlatformIO Serial Plotter
  // reads "label:value" pairs). Motion and alarm are scaled x60/x50 so their
  // 0/1 state is visible on the same scale as HR/SpO2.
  Serial.printf("HR:%.1f,SpO2:%.1f,Motion:%d,Alarm:%d\r\n",
                hrBpm, spo2Pct, motionActive ? 60 : 0, alarmActive ? 50 : 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  pinMode(MOTION_PIN, INPUT);

  connectWiFi();
  connectMQTT();

  // example of using the writable config registers on the AFE
  afeWriteReg8(REG_LED_CURRENT, 60);
  afeWriteReg8(REG_GAIN, 4);

  memset(waveBuf, 18, sizeof(waveBuf));

  // periodic "heartbeat" MQTT message so the HiveMQ screenshot always has
  // a recent, real timestamp even if no alarm has fired yet
  publishAlarm("startup");
}

unsigned long lastHeartbeatMs = 0;
unsigned long lastOledMs = 0;

void loop() {
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // --- read the motion flag, with a short hangover after it clears ---
  bool motionNow = digitalRead(MOTION_PIN) == HIGH;
  unsigned long nowMs = millis();
  if (motionNow) {
    motionActive = true;
    motionHoldUntilMs = nowMs + MOTION_HANGOVER_MS;
  } else if (nowMs > motionHoldUntilMs) {
    motionActive = false;
  }

  uint8_t status = afeReadReg8(REG_STATUS);
  if (status & 0x01) { // DRDY
    uint16_t redCode = afeReadReg16(REG_RED_MSB);
    uint16_t irCode  = afeReadReg16(REG_IR_MSB);
    processSample(redCode, irCode);
  }

  unsigned long now = millis();
  if (now - lastOledMs > 100) {
    lastOledMs = now;
    updateOled();
  }
  if (now - lastHeartbeatMs > 15000) {
    lastHeartbeatMs = now;
    publishAlarm(alarmActive ? "still_in_alarm" : "status_ok");
  }
}
