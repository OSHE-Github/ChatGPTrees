/*
  ChatGPTrees | Arduino IDE-only data collector | UNO Q
  ==================================================
  From the user's working sketch: A0, 14-bit ADC, 3.3 V reference,
  100 ADC conversions/reading, one reading per second.

  A0 = positive tree electrode, GND = negative/reference electrode.
  A1 = OPTIONAL analog soil-moisture sensor (3.3V MAX).
  A3 = OPTIONAL supercapacitor voltage sense (3.3V MAX).
  Qwiic = OPTIONAL LTR390 light sensor (requires Wire, no extra library).

  All voltage logging / load / charge tests run with tree + load
  correctly wired by the experimenter. Load resistor goes BETWEEN
  the electrode contacts (in parallel with A0/GND) -- never from an
  Arduino output pin.

  IMPORTANT: Live measurements occur every second but RAM saves
  timestamped voltage snapshots ~10 seconds apart plus minute summaries.
  All stored data is volatile. Export CSV via Serial Monitor before reset.
  Arduino IDE does NOT create a persistent .csv file automatically.
*/

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// Change to 1 after attaching an Adafruit LTR390 to UNO Q's 3.3V Qwiic.
#define USE_LTR390 0
#if USE_LTR390
#include <Wire.h>
#endif

const int TREE_PIN = A0;
const int SOIL_PIN = A1;
const int CAP_PIN = A3;
const float ADC_REFERENCE_V = 3.300f;
const float ADC_MAX = 16383.0f;             // UNO Q, 14-bit
const uint32_t SAMPLE_EVERY_MS = 1000UL;
const int ADC_SAMPLES = 100;
const uint16_t SNAP_CAPACITY = 9000;       // ~25h of 10-second snapshots
const uint16_t MINUTE_CAPACITY = 1650;       // 1440 minutes + short tests
const uint8_t RUN_CAPACITY = 32;

// One voltage snapshot per ~10 seconds, with its ACTUAL elapsed timestamp.
// Still sample and display every second. This keeps RAM safely below 256 KB.
// sizeof(Snapshot)=8 bytes on the UNO Q (4 elapsedS + 2 adc + padding).
struct Snapshot { uint32_t elapsedS; uint16_t adc; };
static_assert(sizeof(Snapshot) <= 8, "Snapshot storage must be <= 8 bytes");
Snapshot snapshots[SNAP_CAPACITY];
uint16_t snapCount = 0;
uint8_t snapRunId = 0;                  // 0 = no stored voltage run
uint32_t lastSnapElapsed = 0;

// Compact 28-byte record to fit 24h summaries in the UNO Q's 256KB SRAM.
// Missing optional channels are tracked with flags (not sentinel zeros).
struct MinuteRow {
  uint32_t elapsedS;
  float powerMeanUW;
  uint32_t lightRawMean;
  uint16_t count;
  uint16_t adcMean;
  uint16_t minAdc;
  uint16_t maxAdc;
  uint16_t soilAdcMean;
  uint16_t capAdcMean;
  uint8_t runId;
  uint8_t flags;
};
const uint8_t HAS_SOIL = 1, HAS_LIGHT = 2, HAS_CAP = 4;
MinuteRow minutes[MINUTE_CAPACITY];
uint16_t minuteCount = 0;

struct Run {
  uint8_t id;
  char mode;                           // V, L, C
  char tree[24];
  char electrodes[56];
  char startLocal[24];                 // supplied manually; not an RTC
  uint8_t trial;
  float resistanceOhm;
  float capacitanceF;
  float startingCapMV, latestCapMV;
  uint32_t startedAt;
  uint32_t count;
  double adcSum, mvSum, powerSum;
  float minimumMV, maximumMV, maximumPowerUW;
};
Run runs[RUN_CAPACITY];
// Guard against accidental re-introduction of oversized global buffers.
// The board's final Arduino IDE compile report is authoritative.
static_assert(sizeof(snapshots) + sizeof(minutes) + sizeof(runs) < 140000UL,
              "Too many global buffers for 256 KB UNO Q RAM");
uint8_t runCount = 0;
int activeRun = -1;

struct MinuteAccumulator {
  uint16_t count;
  double adcSum, mvSum, powerSum, soilSum, lightSum, capSum;
  uint16_t soilN, lightN, capN;
  float minimumMV, maximumMV;
} acc;

char treeId[24] = "TREE_01";
char electrodeNote[56] = "TRUNK_TO_SOIL";
char startLocal[24] = "UNSET";
char commandLine[128];
uint8_t commandLen = 0;
uint32_t lastTick = 0;
bool liveCsv = false;
bool soilEnabled = false;
bool capEnabled = false;
bool ltrEnabled = false;
float configuredCapacitanceF = 0.0f;

const char *CSV_HEADER =
 "record_type,tree_id,electrode_setup,run_id,mode,trial,load_ohm,start_local,"
 "elapsed_s,samples,raw_adc,voltage_mV,min_mV,max_mV,current_uA,"
 "power_uW,soil_raw_adc,light_raw_als,capacitor_mV,delta_energy_mJ";

#if USE_LTR390
// LTR390 I2C registers; raw ALS counts (NOT calibrated lux).
const uint8_t LTR390_ADDR = 0x53;
bool ltrPresent = false;
bool ltrReadByte(uint8_t address, uint8_t &data) {
  Wire.beginTransmission(LTR390_ADDR);
  Wire.write(address);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LTR390_ADDR, 1) != 1) return false;
  data = (uint8_t)Wire.read();
  return true;
}
bool ltrWriteByte(uint8_t address, uint8_t data) {
  Wire.beginTransmission(LTR390_ADDR);
  Wire.write(address);
  Wire.write(data);
  return Wire.endTransmission() == 0;
}
bool initLTR390() {
  Wire.begin();
  uint8_t id = 0;
  if (!ltrReadByte(0x06, id) || (id & 0xF0) != 0xB0) return false;
  // ALS mode + enable, 18-bit/100 ms default measurement settings.
  return ltrWriteByte(0x00, 0x03);
}
bool readLTR390(uint32_t &rawAls) {
  if (!ltrPresent) return false;
  uint8_t status = 0;
  if (!ltrReadByte(0x07, status) || !(status & 0x08)) return false;
  Wire.beginTransmission(LTR390_ADDR);
  Wire.write((uint8_t)0x0D);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)LTR390_ADDR, 3) != 3) return false;
  uint32_t b0 = (uint8_t)Wire.read();
  uint32_t b1 = (uint8_t)Wire.read();
  uint32_t b2 = (uint8_t)Wire.read();
  rawAls = b0 | (b1 << 8) | (b2 << 16);
  return true;
}
#endif

void copySafe(char *dest, size_t cap, const char *src) {
  size_t j = 0;
  while (*src && j + 1 < cap) {
    char c = *src++;
    // No commas / quotes / line breaks in free-form CSV fields.
    if (isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.' ||
        c == ':' || c == 'T' || c == '+') dest[j++] = c;
    else if (c == ' ') dest[j++] = '_';
  }
  dest[j] = '\0';
}
bool equalWord(const char *a, const char *b) {
  while (*a && *b) {
    if (toupper((unsigned char)*a++) != toupper((unsigned char)*b++)) return false;
  }
  return *a == 0 && *b == 0;
}
const char *modeName(char c) {
  return c == 'V' ? "VOLTAGE" : c == 'L' ? "LOAD" : "CHARGE";
}
float mvFromAdc(float adc) {
  return adc * (ADC_REFERENCE_V * 1000.0f / ADC_MAX);
}
uint16_t storedAdc(float adc) {
  if (adc < 0.0f) return 0;
  if (adc > ADC_MAX) return (uint16_t)ADC_MAX;
  return (uint16_t)(adc + 0.5f);
}
uint16_t adcFromMv(float mv) {
  return storedAdc(mv * ADC_MAX / (ADC_REFERENCE_V * 1000.0f));
}
float readAveragedAdc(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    sum += (uint32_t)analogRead(pin);
    delay(2);
  }
  return (float)sum / (float)ADC_SAMPLES;
}
void resetAccumulator() {
  memset(&acc, 0, sizeof(acc));
  acc.minimumMV = 1.0e9f;
  acc.maximumMV = -1.0e9f;
}
void csvPrefix(const char *kind, const Run &r) {
  Serial.print(kind); Serial.print(',');
  Serial.print(r.tree); Serial.print(',');
  Serial.print(r.electrodes); Serial.print(',');
  Serial.print(r.id); Serial.print(',');
  Serial.print(modeName(r.mode)); Serial.print(',');
  if (r.mode == 'L') Serial.print(r.trial);
  Serial.print(',');
  if (r.mode == 'L') Serial.print(r.resistanceOhm, 2);
  Serial.print(',');
  Serial.print(r.startLocal); Serial.print(',');
}
float energyChangeMJ(const Run &r, float capMV) {
  if (r.mode != 'C' || r.capacitanceF <= 0) return 0;
  float initialV = r.startingCapMV / 1000.0f;
  float nowV = capMV / 1000.0f;
  return 500.0f * r.capacitanceF * (nowV * nowV - initialV * initialV);
}
void printMeasurements(float adc, float mv, float mn, float mx, float p,
                       bool hasSoil, float soil, bool hasLight, float light,
                       bool hasCap, float capMv, float energyMJ,
                       const Run &r) {
  Serial.print(adc, 2); Serial.print(',');
  Serial.print(mv, 3); Serial.print(',');
  Serial.print(mn, 3); Serial.print(',');
  Serial.print(mx, 3); Serial.print(',');
  if (r.mode == 'L') Serial.print(mv * 1000.0f / r.resistanceOhm, 5);
  Serial.print(',');
  if (r.mode == 'L') Serial.print(p, 6);
  Serial.print(',');
  if (hasSoil) Serial.print(soil, 2);
  Serial.print(',');
  if (hasLight) Serial.print(light, 0);
  Serial.print(',');
  if (hasCap) Serial.print(capMv, 3);
  Serial.print(',');
  if (r.mode == 'C' && hasCap && r.capacitanceF > 0) Serial.print(energyMJ, 6);
  Serial.println();
}
void printCsvLive(const Run &r, uint32_t elapsedS, float adc, float mv,
                  float p, bool hasSoil, float soil, bool hasLight, float light,
                  bool hasCap, float cap) {
  csvPrefix("SAMPLE", r);
  Serial.print(elapsedS); Serial.print(",1,");
  printMeasurements(adc, mv, mv, mv, p, hasSoil, soil,
                    hasLight, light, hasCap, cap, energyChangeMJ(r, cap), r);
}
void printMinuteCsv(const MinuteRow &m) {
  const Run &r = runs[m.runId - 1];
  csvPrefix("MINUTE", r);
  Serial.print(m.elapsedS); Serial.print(',');
  Serial.print(m.count); Serial.print(',');
  float capMv = mvFromAdc(m.capAdcMean);
  printMeasurements(m.adcMean, mvFromAdc(m.adcMean),
                    mvFromAdc(m.minAdc), mvFromAdc(m.maxAdc), m.powerMeanUW,
                    (m.flags & HAS_SOIL) != 0, m.soilAdcMean,
                    (m.flags & HAS_LIGHT) != 0, m.lightRawMean,
                    (m.flags & HAS_CAP) != 0, capMv,
                    energyChangeMJ(r, capMv), r);
}
void flushMinute() {
  if (activeRun < 0 || acc.count == 0) return;
  Run &r = runs[activeRun];
  if (minuteCount >= MINUTE_CAPACITY) {
    Serial.println("# WARNING: minute buffer full; stop and export ASAP.");
    resetAccumulator();
    return;
  }
  MinuteRow &m = minutes[minuteCount++];
  m.runId = r.id;
  m.count = acc.count;
  m.elapsedS = (millis() - r.startedAt) / 1000UL;
  m.adcMean = storedAdc((float)(acc.adcSum / acc.count));
  m.minAdc = adcFromMv(acc.minimumMV);
  m.maxAdc = adcFromMv(acc.maximumMV);
  m.powerMeanUW = (float)(acc.powerSum / acc.count);
  m.flags = 0;
  m.soilAdcMean = 0;
  m.lightRawMean = 0;
  m.capAdcMean = 0;
  if (acc.soilN) {
    m.flags |= HAS_SOIL;
    m.soilAdcMean = storedAdc((float)(acc.soilSum / acc.soilN));
  }
  if (acc.lightN) {
    m.flags |= HAS_LIGHT;
    m.lightRawMean = (uint32_t)(acc.lightSum / acc.lightN + 0.5);
  }
  if (acc.capN) {
    m.flags |= HAS_CAP;
    m.capAdcMean = adcFromMv((float)(acc.capSum / acc.capN));
  }
  if (liveCsv) printMinuteCsv(m);
  else {
    Serial.print("# Minute saved: run="); Serial.print(r.id);
    Serial.print(" count="); Serial.print(m.count);
    Serial.print(" mean_mV="); Serial.println(mvFromAdc(m.adcMean), 3);
  }
  resetAccumulator();
}
void stopRun() {
  if (activeRun < 0) { Serial.println("# No active test."); return; }
  flushMinute();
  const Run &r = runs[activeRun];
  Serial.print("# STOP run="); Serial.print(r.id);
  Serial.print(" samples="); Serial.println(r.count);
  if (r.mode == 'C' && capEnabled && r.capacitanceF > 0) {
    float delta = energyChangeMJ(r, r.latestCapMV);
    Serial.print("# Charge duration_min=");
    Serial.print((millis() - r.startedAt) / 60000.0f, 2);
    Serial.print(" initial_mV="); Serial.print(r.startingCapMV, 3);
    Serial.print(" final_mV="); Serial.print(r.latestCapMV, 3);
    Serial.print(" delta_energy_mJ="); Serial.println(delta, 6);
    Serial.println("# Charging proof also requires the tree to be the ONLY energy input.");
  }
  activeRun = -1;
}
void startRun(char mode, float resistor = 0, int trial = 0) {
  if (activeRun >= 0) {
    Serial.println("# ERROR: STOP the current test first."); return;
  }
  if (runCount >= RUN_CAPACITY) {
    Serial.println("# ERROR: run buffer full; export before CLEAR."); return;
  }
  if (mode == 'V' && snapRunId != 0) {
    Serial.println("# ERROR: Snapshot memory already holds a voltage run.");
    Serial.println("# Export it before CLEAR. LOAD and CHARGE tests are still permitted.");
    return;
  }
  if (mode == 'C' && !capEnabled) {
    Serial.println("# ERROR: Enable capacitor sensing first: CAP ON"); return;
  }
  if (mode == 'C' && configuredCapacitanceF <= 0) {
    Serial.println("# ERROR: Set capacitor size in farads: CAPF 0.47"); return;
  }
  if (mode == 'L' && (resistor < 1.0f || trial < 1 || trial > 99)) {
    Serial.println("# ERROR: START LOAD <resistance_ohms> <trial_1_to_99>"); return;
  }
  Run &r = runs[runCount];
  memset(&r, 0, sizeof(r));
  r.id = runCount + 1;
  r.mode = mode;
  copySafe(r.tree, sizeof(r.tree), treeId);
  copySafe(r.electrodes, sizeof(r.electrodes), electrodeNote);
  copySafe(r.startLocal, sizeof(r.startLocal), startLocal);
  r.resistanceOhm = resistor;
  r.trial = (uint8_t)trial;
  r.capacitanceF = configuredCapacitanceF;
  r.minimumMV = 1.0e9f;
  r.maximumMV = -1.0e9f;
  r.startedAt = millis();
  if (mode == 'C') {
    r.startingCapMV = mvFromAdc(readAveragedAdc(CAP_PIN));
    r.latestCapMV = r.startingCapMV;
  }
  activeRun = runCount++;
  if (mode == 'V') { snapRunId = r.id; lastSnapElapsed = 0; }
  resetAccumulator();
  lastTick = millis();
  Serial.print("# START run="); Serial.print(r.id);
  Serial.print(" mode="); Serial.println(modeName(mode));
  Serial.println("# Timestamp = start_local + elapsed_s; start_local is manually entered.");
  if (mode == 'L') Serial.println("# IMPORTANT: resistor must be physically connected across electrodes.");
}
void takeSample() {
  if (activeRun < 0) return;
  Run &r = runs[activeRun];
  if (minuteCount >= MINUTE_CAPACITY) {
    Serial.println("# Minute summary buffer full: stopping to preserve existing data.");
    stopRun();
    return;
  }
  // Preserve the original proven voltage acquisition settings.
  float adc = readAveragedAdc(TREE_PIN);
  float mv = mvFromAdc(adc);
  float p = (r.mode == 'L') ? (mv * mv / r.resistanceOhm) : 0.0f;
  uint32_t seconds = (millis() - r.startedAt) / 1000UL;

  bool gotSoil = false, gotLight = false, gotCap = false;
  float soil = 0, light = 0, cap = 0;
  if (soilEnabled) { soil = readAveragedAdc(SOIL_PIN); gotSoil = true; }
#if USE_LTR390
  if (ltrEnabled && ltrPresent) {
    uint32_t raw = 0;
    gotLight = readLTR390(raw);
    if (gotLight) light = (float)raw;
  }
#endif
  if (capEnabled) {
    cap = mvFromAdc(readAveragedAdc(CAP_PIN));
    gotCap = true;
    if (r.mode == 'C') r.latestCapMV = cap;
  }

  r.count++;
  r.adcSum += adc;
  r.mvSum += mv;
  r.powerSum += p;
  if (mv < r.minimumMV) r.minimumMV = mv;
  if (mv > r.maximumMV) r.maximumMV = mv;
  if (p > r.maximumPowerUW) r.maximumPowerUW = p;

  acc.count++;
  acc.adcSum += adc;
  acc.mvSum += mv;
  acc.powerSum += p;
  if (mv < acc.minimumMV) acc.minimumMV = mv;
  if (mv > acc.maximumMV) acc.maximumMV = mv;
  if (gotSoil) { acc.soilSum += soil; acc.soilN++; }
  if (gotLight) { acc.lightSum += light; acc.lightN++; }
  if (gotCap) { acc.capSum += cap; acc.capN++; }

  if (r.mode == 'V' &&
      (snapCount == 0 || seconds - lastSnapElapsed >= 10UL)) {
    if (snapCount < SNAP_CAPACITY) {
      snapshots[snapCount].elapsedS = seconds;
      snapshots[snapCount].adc = storedAdc(adc);
      ++snapCount;
      lastSnapElapsed = seconds;
    } else {
      Serial.println("# WARNING: Snapshot buffer full; voltage run stopping.");
      stopRun();
      return;
    }
  }

  if (liveCsv) printCsvLive(r, seconds, adc, mv, p, gotSoil, soil,
                            gotLight, light, gotCap, cap);
  else {
    Serial.print("# LIVE run="); Serial.print(r.id);
    Serial.print(" elapsed_s="); Serial.print(seconds);
    Serial.print(" adc="); Serial.print(adc, 2);
    Serial.print(" mV="); Serial.print(mv, 3);
    if (r.mode == 'L') {
      Serial.print(" uA="); Serial.print(mv * 1000.0f / r.resistanceOhm, 5);
      Serial.print(" uW="); Serial.print(p, 6);
    }
    if (gotSoil) { Serial.print(" soil_adc="); Serial.print(soil, 1); }
    if (gotLight) { Serial.print(" ALS_raw="); Serial.print(light, 0); }
    if (gotCap) { Serial.print(" cap_mV="); Serial.print(cap, 2); }
    Serial.println();
  }
  if (acc.count >= 60) flushMinute();
}
// Export saved ~10-second voltage snapshots. Each has an independently
// recorded elapsed time; no interpolation or invented readings.
void printSnapshotRange(uint32_t first, uint32_t amount) {
  if (snapRunId == 0) { Serial.println("# No voltage SNAPSHOT data recorded."); return; }
  if (first >= snapCount) { Serial.println("# Start index is beyond recorded data."); return; }
  if (amount < 1 || amount > 200) amount = 100;
  uint32_t end = first + amount;
  if (end > snapCount) end = snapCount;
  const Run &r = runs[snapRunId - 1];
  Serial.println(CSV_HEADER);
  for (uint32_t i = first; i < end; ++i) {
    float adc = (float)snapshots[i].adc;
    float mv = mvFromAdc(adc);
    csvPrefix("SNAPSHOT", r);
    Serial.print(snapshots[i].elapsedS); Serial.print(",1,");
    printMeasurements(adc, mv, mv, mv, 0, false, 0, false, 0,
                      false, 0, 0, r);
  }
  Serial.print("# SNAPSHOT exported indices ["); Serial.print(first);
  Serial.print(".. "); Serial.print(end - 1);
  Serial.print("]; next_index="); Serial.println(end);
}
void printMinuteRange(uint32_t first, uint32_t amount) {
  if (first >= minuteCount) { Serial.println("# No minute rows in that range."); return; }
  if (amount < 1 || amount > 200) amount = 100;
  uint32_t end = first + amount;
  if (end > minuteCount) end = minuteCount;
  Serial.println(CSV_HEADER);
  for (uint32_t i = first; i < end; ++i) printMinuteCsv(minutes[i]);
  Serial.print("# MINUTE exported indices ["); Serial.print(first);
  Serial.print(".. "); Serial.print(end - 1);
  Serial.print("]; next_index="); Serial.println(end);
}
void report() {
  Serial.println("# RUN REPORT (mean power = average of V^2/R, not mean(V)^2/R)");
  for (uint8_t i = 0; i < runCount; ++i) {
    const Run &r = runs[i];
    Serial.print("# run="); Serial.print(r.id);
    Serial.print(" tree="); Serial.print(r.tree);
    Serial.print(" mode="); Serial.print(modeName(r.mode));
    Serial.print(" samples="); Serial.print(r.count);
    if (r.count > 0) {
      Serial.print(" mean_mV="); Serial.print((float)(r.mvSum / r.count), 3);
      Serial.print(" min_mV="); Serial.print(r.minimumMV, 3);
      Serial.print(" max_mV="); Serial.print(r.maximumMV, 3);
    }
    if (r.mode == 'L') {
      Serial.print(" R_ohm="); Serial.print(r.resistanceOhm, 2);
      Serial.print(" trial="); Serial.print(r.trial);
      if (r.count) {
        Serial.print(" mean_uW="); Serial.print((float)(r.powerSum / r.count), 6);
        Serial.print(" max_uW="); Serial.print(r.maximumPowerUW, 6);
      }
    }
    if (r.mode == 'C') {
      Serial.print(" cap_start_mV="); Serial.print(r.startingCapMV, 2);
      Serial.print(" cap_latest_mV="); Serial.print(r.latestCapMV, 2);
      Serial.print(" energy_change_mJ="); Serial.print(energyChangeMJ(r, r.latestCapMV), 6);
    }
    Serial.println();
  }
  float maxP = -1;
  uint8_t maxRun = 0;
  uint8_t distinctValidLoads = 0;
  for (uint8_t i = 0; i < runCount; ++i) {
    const Run &r = runs[i];
    if (r.mode != 'L' || r.count == 0) continue;
    bool alreadySeen = false;
    for (uint8_t k = 0; k < i; ++k)
      if (runs[k].mode == 'L' && runs[k].resistanceOhm == r.resistanceOhm)
        alreadySeen = true;
    if (alreadySeen) continue;
    uint8_t trialIds[RUN_CAPACITY]; uint8_t n = 0;
    for (uint8_t j = i; j < runCount; ++j) {
      const Run &t = runs[j];
      if (t.mode != 'L' || t.count == 0 || t.resistanceOhm != r.resistanceOhm) continue;
      bool repeated = false;
      for (uint8_t q = 0; q < n; ++q) if (trialIds[q] == t.trial) repeated = true;
      if (!repeated) trialIds[n++] = t.trial;
      if (t.maximumPowerUW > maxP) { maxP = t.maximumPowerUW; maxRun = t.id; }
    }
    if (n >= 3) ++distinctValidLoads;
    Serial.print("# load_ohm="); Serial.print(r.resistanceOhm, 2);
    Serial.print(" distinct_trial_ids="); Serial.println(n);
  }
  Serial.print("# Loads with at least 3 unique trials: ");
  Serial.print(distinctValidLoads); Serial.println(" / 5 required");
  if (maxRun) {
    Serial.print("# Highest measured sample power: "); Serial.print(maxP, 6);
    Serial.print(" uW (run "); Serial.print(maxRun); Serial.println(")");
  }
}
void status() {
  Serial.print("# tree="); Serial.print(treeId);
  Serial.print(" electrodes="); Serial.print(electrodeNote);
  Serial.print(" start_local="); Serial.println(startLocal);
  Serial.print("# active_run=");
  Serial.print(activeRun >= 0 ? runs[activeRun].id : 0);
  Serial.print(" runs="); Serial.print(runCount);
  Serial.print(" snapshots_10sec="); Serial.print(snapCount); Serial.print('/');
  Serial.print(SNAP_CAPACITY);
  Serial.print(" minutes="); Serial.print(minuteCount); Serial.print('/');
  Serial.println(MINUTE_CAPACITY);
  Serial.print("# static_buffers_bytes=");
  Serial.print((uint32_t)(sizeof(snapshots) + sizeof(minutes) + sizeof(runs)));
  Serial.println(" (includes reserved capacity)");
  Serial.print("# soil="); Serial.print(soilEnabled ? "ON" : "OFF");
  Serial.print(" LTR390="); Serial.print(ltrEnabled ? "ON" : "OFF");
  Serial.print(" cap="); Serial.print(capEnabled ? "ON" : "OFF");
  Serial.print(" cap_farads="); Serial.print(configuredCapacitanceF, 6);
  Serial.print(" live_csv="); Serial.println(liveCsv ? "ON" : "OFF");
}
void help() {
  Serial.println("# ==== ChatGPTrees Arduino-only HELP ====");
  Serial.println("# Set Serial Monitor to 115200 baud and Newline ending.");
  Serial.println("# TREE TREE_01                Set Tree ID");
  Serial.println("# ELECTRODES TRUNK_TO_SOIL    Set electrode description (no spaces)");
  Serial.println("# TIME 2026-10-08T14:30:00   Manually record local START timestamp");
  Serial.println("# START VOLTAGE               Live 1-s readings + 10-s saved snapshots");
  Serial.println("# START LOAD 100000 1         Resistor (ohms) + trial ID (1 to 99)");
  Serial.println("# SOIL ON / SOIL OFF          Use optional A1 soil sensor");
  Serial.println("# LIGHT ON / LIGHT OFF        Use optional LTR390 (compile switch)");
  Serial.println("# CAP ON / CAP OFF            Use optional A3 capacitor voltage");
  Serial.println("# CAPF 0.47                   Set measured capacitance in farads");
  Serial.println("# START CHARGE                Log cap voltage/energy change");
  Serial.println("# CSV ON / CSV OFF            One-second CSV live print on/off");
  Serial.println("# STOP                        End test, retain records in RAM");
  Serial.println("# STATUS / REPORT             Buffer usage / summary and load comparison");
  Serial.println("# EXPORT MIN 0 100            Export minute rows 0-99 as CSV");
  Serial.println("# EXPORT SNAP 0 100           Export saved 10-s voltage snapshots");
  Serial.println("# CLEAR YES                   Erase ALL results from RAM (irreversible)");
  Serial.println("# Recommended: STOP, EXPORT in pages, save CSV before reset!");
}
void handleCommand(char *line) {
  char *verb = strtok(line, " \t");
  if (!verb) return;
  char *a = strtok(nullptr, " \t");
  char *b = strtok(nullptr, " \t");
  char *c = strtok(nullptr, " \t");
  if (equalWord(verb, "HELP")) { help(); return; }
  if (equalWord(verb, "STATUS")) { status(); return; }
  if (equalWord(verb, "REPORT")) { report(); return; }
  if (equalWord(verb, "STOP")) { stopRun(); return; }
  if (equalWord(verb, "TREE") && a) {
    if (activeRun >= 0) { Serial.println("# STOP test before changing tree."); return; }
    copySafe(treeId, sizeof(treeId), a);
    status(); return;
  }
  if (equalWord(verb, "ELECTRODES") && a) {
    if (activeRun >= 0) { Serial.println("# STOP test before changing electrodes."); return; }
    copySafe(electrodeNote, sizeof(electrodeNote), a);
    status(); return;
  }
  if (equalWord(verb, "TIME") && a) {
    if (activeRun >= 0) { Serial.println("# STOP test before changing timestamp."); return; }
    if (strlen(a) != 19 || a[4] != '-' || a[7] != '-' || a[10] != 'T' ||
        a[13] != ':' || a[16] != ':') {
      Serial.println("# Use TIME YYYY-MM-DDTHH:MM:SS (your local clock)."); return;
    }
    copySafe(startLocal, sizeof(startLocal), a);
    status(); return;
  }
  if (equalWord(verb, "START") && a) {
    if (equalWord(a, "VOLTAGE")) { startRun('V'); return; }
    if (equalWord(a, "CHARGE")) { startRun('C'); return; }
    if (equalWord(a, "LOAD") && b && c) {
      char *end = nullptr;
      float ohms = strtof(b, &end);
      if (!end || *end != '\0') { Serial.println("# Invalid load resistance."); return; }
      long trial = strtol(c, &end, 10);
      if (!end || *end != '\0') { Serial.println("# Invalid trial ID."); return; }
      startRun('L', ohms, (int)trial); return;
    }
    Serial.println("# Usage: START VOLTAGE | START LOAD <ohms> <trial> | START CHARGE");
    return;
  }
  if (equalWord(verb, "CSV") && a) {
    if (equalWord(a, "ON")) { liveCsv = true; Serial.println(CSV_HEADER); return; }
    if (equalWord(a, "OFF")) { liveCsv = false; Serial.println("# CSV output OFF"); return; }
  }
  if (equalWord(verb, "SOIL") && a) {
    soilEnabled = equalWord(a, "ON"); status(); return;
  }
  if (equalWord(verb, "LIGHT") && a) {
#if USE_LTR390
    ltrEnabled = equalWord(a, "ON");
    if (ltrEnabled) {
      ltrPresent = initLTR390();
      if (!ltrPresent) { ltrEnabled = false; Serial.println("# LTR390 not found at 0x53."); }
    }
#else
    ltrEnabled = false;
    Serial.println("# LTR390 disabled at compile time; set USE_LTR390 to 1 first.");
#endif
    status(); return;
  }
  if (equalWord(verb, "CAP") && a) {
    capEnabled = equalWord(a, "ON"); status(); return;
  }
  if (equalWord(verb, "CAPF") && a) {
    float farads = strtof(a, nullptr);
    if (farads <= 0 || farads > 100000.0f) {
      Serial.println("# CAPF must be positive farads, e.g. CAPF 0.47"); return;
    }
    configuredCapacitanceF = farads;
    status(); return;
  }
  if (equalWord(verb, "EXPORT") && a && b && c) {
    if (activeRun >= 0) {
      Serial.println("# STOP first: exporting while measuring causes timing gaps."); return;
    }
    uint32_t first = (uint32_t)strtoul(b, nullptr, 10);
    uint32_t count = (uint32_t)strtoul(c, nullptr, 10);
    if (equalWord(a, "SNAP")) { printSnapshotRange(first, count); return; }
    if (equalWord(a, "MIN")) { printMinuteRange(first, count); return; }
  }
  if (equalWord(verb, "CLEAR") && a && equalWord(a, "YES")) {
    if (activeRun >= 0) { Serial.println("# STOP first."); return; }
    snapCount = 0; snapRunId = 0; lastSnapElapsed = 0;
    minuteCount = 0; runCount = 0;
    Serial.println("# ALL buffered data erased; CSV exports cannot recover it."); return;
  }
  Serial.println("# Unknown command. Type HELP.");
}
void readCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (commandLen > 0) {
        commandLine[commandLen] = '\0';
        handleCommand(commandLine);
        commandLen = 0;
      }
    } else if (commandLen < sizeof(commandLine) - 1) {
      commandLine[commandLen++] = c;
    } else {
      commandLen = 0;
      Serial.println("# Command too long; retry.");
    }
  }
}
void setup() {
  Serial.begin(115200);
  delay(2000);
  analogReadResolution(14);
  resetAccumulator();
  Serial.println("# ChatGPTrees Arduino IDE-only logger ready. Type HELP.");
  Serial.println("# IMPORTANT: data only in RAM; Arduino Serial Monitor does NOT save CSV files.");
  Serial.println("# Live = ~1 sec; saved SNAPSHOT = ~10 sec; saved MINUTE = 60 samples.");
  Serial.println("# Analog inputs MUST remain between 0 and 3.3V; don't apply 5V.");
  status();
  lastTick = millis();
}
void loop() {
  readCommands();
  uint32_t now = millis();
  if (activeRun >= 0 && (uint32_t)(now - lastTick) >= SAMPLE_EVERY_MS) {
    lastTick = now;                  // no catch-up samples after serial export
    takeSample();
  }
}
