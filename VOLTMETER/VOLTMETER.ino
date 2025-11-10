/******************************************************************
  VoltMeter — Memory-optimized build for Arduino Uno (ATmega328P)
  - Reduces SRAM footprint so data section fits on Uno
  - Keeps robust sampling & calibration features
******************************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>

// ================== Configuration Constants ==================
#define FW_VERSION "VoltMeter v1.2 (mem-opt)"
#define SERIAL_BAUD 115200
#define ANALOG_PIN A0
#define DEFAULT_LOG_INTERVAL_MS 1000UL
// Reduced default cal samples to be more realistic and RAM-friendly
#define DEFAULT_CAL_SAMPLES 256
// Reduced medians to cut SRAM usage. 64 -> medians/deviations/kept arrays use ~384 bytes each as stack locals.
#define MAX_MEDIANS 64
#define DISCARD_AFTER_REF_SWITCH 5
#define IIR_ALPHA 0.90f
#define VREF_DEFAULT_VOLTS 5.00f
#define VREF_INTERNAL_VOLTS 1.10f

// EEPROM layout addresses
#define EEPROM_BOOT_COUNTER_ADDR 0
#define EEPROM_CALIB_ADDR 16

// Magic signature for calibration block: 'VOLT'
#define CAL_MAGIC 0x564F4C54UL
#define CAL_VERSION 0x0002

// ADC reference selection modes
enum RefMode : uint8_t { REF_AUTO = 0, REF_DEFAULT = 1, REF_INTERNAL = 2 };

// ================== Data Structures ==================
struct CalData {
  uint32_t magic;      // CAL_MAGIC
  uint16_t version;    // CAL_VERSION
  uint8_t adcRef;      // RefMode used during calibration
  uint16_t sampleCount;
  float vrefUsed;
  float K;
  float rTop;
  float rBot;
  float adcMean;
  float adcStddev;
  uint32_t calMillis;
  uint32_t calCounter;
  uint16_t crc16;
} __attribute__((packed));

// Note: keeping CalData as-is — it is small relative to the big arrays.

// Stats
struct Stats {
  float mean;
  float stddev;
  uint16_t kept;
  uint16_t total;
  uint16_t rawMin;
  uint16_t rawMax;
};

// ================== Globals (kept small) ==================
static CalData gCal;
static bool gCalValid = false;
static uint32_t gBootCounter = 0;
static RefMode gRefMode = REF_AUTO;
static RefMode gActiveRef = REF_DEFAULT;
static bool gLoggingEnabled = true;
static uint32_t gLogIntervalMs = DEFAULT_LOG_INTERVAL_MS;
static uint32_t gLastLogMs = 0;
static float gIIRVin = 0.0f;

// Small serial buffer (reduced)
static char gSerialBuf[64];

// ================== Utility: CRC16-CCITT ==================
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else crc = (crc << 1);
    }
  }
  return crc;
}

// ================== EEPROM Helpers ==================
static void eepromReadBootCounter() {
  uint32_t val = 0xFFFFFFFFUL;
  EEPROM.get(EEPROM_BOOT_COUNTER_ADDR, val);
  if (val == 0xFFFFFFFFUL) gBootCounter = 0;
  else gBootCounter = val;
}

static void eepromIncrementBootCounter() {
  gBootCounter++;
  EEPROM.put(EEPROM_BOOT_COUNTER_ADDR, gBootCounter);
}

static bool eepromLoadCalibration() {
  CalData stored;
  EEPROM.get(EEPROM_CALIB_ADDR, stored);
  if (stored.magic != CAL_MAGIC || stored.version != CAL_VERSION) return false;
  CalData tmp = stored;
  tmp.crc16 = 0;
  uint16_t crc = crc16_ccitt((const uint8_t*)&tmp, sizeof(CalData));
  if (crc != stored.crc16) {
    Serial.println(F("[EEPROM] Calibration CRC FAILED."));
    return false;
  }
  gCal = stored;
  return true;
}

static bool eepromSaveCalibration(const CalData& cd) {
  CalData tmp = cd;
  tmp.crc16 = 0;
  uint16_t crc = crc16_ccitt((const uint8_t*)&tmp, sizeof(CalData));
  tmp.crc16 = crc;
  EEPROM.put(EEPROM_CALIB_ADDR, tmp);
  CalData verify;
  EEPROM.get(EEPROM_CALIB_ADDR, verify);
  bool ok = (verify.magic == CAL_MAGIC) && (verify.version == CAL_VERSION) && (verify.crc16 == crc);
  Serial.print(F("[EEPROM] Save CRC: 0x")); Serial.println(crc, HEX);
  Serial.println(ok ? F("[EEPROM] Save OK") : F("[EEPROM] Save FAILED"));
  return ok;
}

static void eepromClearCalibration() {
  CalData blank;
  memset(&blank, 0, sizeof(blank));
  EEPROM.put(EEPROM_CALIB_ADDR, blank);
  gCalValid = false;
}

// ================== ADC Reference Control ==================
static float refVolts(RefMode m) {
  switch (m) {
    case REF_INTERNAL: return VREF_INTERNAL_VOLTS;
    case REF_DEFAULT:  return VREF_DEFAULT_VOLTS;
    case REF_AUTO:     return refVolts(gActiveRef);
  }
  return VREF_DEFAULT_VOLTS;
}

static void applyReference(RefMode m) {
  RefMode prev = gActiveRef;
  switch (m) {
    case REF_INTERNAL: analogReference(INTERNAL); gActiveRef = REF_INTERNAL; break;
    case REF_DEFAULT:  analogReference(DEFAULT);  gActiveRef = REF_DEFAULT; break;
    case REF_AUTO: break;
  }
  if (prev != gActiveRef) {
    for (int i = 0; i < DISCARD_AFTER_REF_SWITCH; ++i) { analogRead(ANALOG_PIN); delay(2); }
  }
}

// ================== Robust Sampling (memory-reduced) ==================
static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

static Stats robustSample(uint16_t totalSamples) {
  if (totalSamples < 3) totalSamples = 3;
  if (totalSamples > (MAX_MEDIANS * 3)) totalSamples = (MAX_MEDIANS * 3);
  const uint16_t groups = totalSamples / 3;

  // Local arrays on stack sized by MAX_MEDIANS (much smaller now)
  uint16_t medians[MAX_MEDIANS];
  uint16_t deviations[MAX_MEDIANS];
  uint16_t keptVals[MAX_MEDIANS];

  uint16_t rawMin = 65535, rawMax = 0;

  // First pass: collect medians of groups of 3
  for (uint16_t i = 0; i < groups; ++i) {
    uint16_t a = analogRead(ANALOG_PIN); delayMicroseconds(300);
    uint16_t b = analogRead(ANALOG_PIN); delayMicroseconds(300);
    uint16_t c = analogRead(ANALOG_PIN);
    uint16_t m = median3(a, b, c);
    medians[i] = m;
    if (a < rawMin) rawMin = a; if (a > rawMax) rawMax = a;
    if (b < rawMin) rawMin = b; if (b > rawMax) rawMax = b;
    if (c < rawMin) rawMin = c; if (c > rawMax) rawMax = c;
  }

  // Sort medians (insertion sort)
  for (uint16_t i = 1; i < groups; ++i) {
    uint16_t key = medians[i];
    int j = i - 1;
    while (j >= 0 && medians[j] > key) { medians[j + 1] = medians[j]; --j; }
    medians[j + 1] = key;
  }
  uint16_t med = medians[groups / 2];

  // deviations and MAD
  for (uint16_t i = 0; i < groups; ++i) {
    uint16_t d = (medians[i] > med) ? (medians[i] - med) : (med - medians[i]);
    deviations[i] = d;
  }
  // sort deviations
  for (uint16_t i = 1; i < groups; ++i) {
    uint16_t key = deviations[i];
    int j = i - 1;
    while (j >= 0 && deviations[j] > key) { deviations[j + 1] = deviations[j]; --j; }
    deviations[j + 1] = key;
  }
  uint16_t MAD = deviations[groups / 2];
  float thr = (float)MAD * 3.0f;

  // Second pass: collect kept medians (trimmed)
  uint16_t kept = 0;
  for (uint16_t i = 0; i < groups; ++i) {
    uint16_t a = analogRead(ANALOG_PIN); delayMicroseconds(300);
    uint16_t b = analogRead(ANALOG_PIN); delayMicroseconds(300);
    uint16_t c = analogRead(ANALOG_PIN);
    uint16_t m2 = median3(a, b, c);
    float dev = fabs((float)m2 - (float)med);
    if (dev <= thr) {
      if (kept < MAX_MEDIANS) keptVals[kept++] = m2; // safety limit
    }
  }

  float mean = 0.0f;
  float stddev = 0.0f;
  if (kept > 0) {
    double sum = 0.0;
    for (uint16_t i = 0; i < kept; ++i) sum += (double)keptVals[i];
    mean = (float)(sum / (double)kept);
    if (kept > 1) {
      double sumsq = 0.0;
      for (uint16_t i = 0; i < kept; ++i) {
        double d = (double)keptVals[i] - (double)mean;
        sumsq += d * d;
      }
      stddev = (float)sqrt(sumsq / (double)(kept - 1));
    } else stddev = 0.0f;
  } else {
    mean = (float)med;
    stddev = (float)MAD * 1.4826f; // approx conversion
  }

  Stats st;
  st.mean = mean;
  st.stddev = stddev;
  st.kept = kept;
  st.total = groups * 3;
  st.rawMin = rawMin;
  st.rawMax = rawMax;
  return st;
}

// ================== Reference Auto-Selection ==================
static void selectReferenceAuto() {
  applyReference(REF_DEFAULT);
  Stats st = robustSample(30);
  if (st.mean < 250.0f) applyReference(REF_INTERNAL);
}

// ================== Measurement & Reporting ==================
static void printPhysicalLimitationStatement() {
  Serial.println(F("[Note] From a single ADC measurement you can only compute the resistor divider ratio."));
}

static void printStartupReport() {
  Serial.println();
  Serial.println(F("================= Voltmeter Startup ================="));
  Serial.print(F("Firmware: ")); Serial.println(FW_VERSION);
  Serial.print(F("Build : ")); Serial.print(__DATE__); Serial.print(F(" ")); Serial.println(__TIME__);
  Serial.print(F("Boot Cnt: ")); Serial.println(gBootCounter);
  if (gCalValid) {
    Serial.println(F("EEPROM: Calibration VALID"));
    Serial.print(F(" Version : ")); Serial.println(gCal.version);
    Serial.print(F(" Ref Used : ")); Serial.println(gCal.adcRef == REF_INTERNAL ? F("INTERNAL 1.1V") : F("DEFAULT Vcc"));
    Serial.print(F(" Vref Volts : ")); Serial.println(gCal.vrefUsed, 6);
    Serial.print(F(" Samples : ")); Serial.println(gCal.sampleCount);
    Serial.print(F(" ADC Mean : ")); Serial.println(gCal.adcMean, 4);
    Serial.print(F(" ADC Stddev : ")); Serial.println(gCal.adcStddev, 4);
    Serial.print(F(" Ratio K : ")); Serial.println(gCal.K, 8);
    Serial.print(F(" RTOP est : ")); Serial.println(gCal.rTop, 2);
    Serial.print(F(" RBOT est : ")); Serial.println(gCal.rBot, 2);
    Serial.print(F(" CalMillis : ")); Serial.println(gCal.calMillis);
    Serial.print(F(" CalCounter : ")); Serial.println(gCal.calCounter);
    Serial.print(F(" CRC16 : 0x")); Serial.println(gCal.crc16, HEX);
  } else {
    Serial.println(F("EEPROM: No valid calibration (CRC fail or empty)."));
    Serial.println(F(" Run CAL <volts> [RBOT <ohms>|RTOP <ohms>] [SAMPLES <n>]"));
  }
  printPhysicalLimitationStatement();
  Serial.println(F("====================================================="));
}

static float currentVrefVolts() { return refVolts(gActiveRef); }

static void printMeasurementLine(const Stats& st) {
  const uint32_t t = millis();
  const float vref = currentVrefVolts();
  const float vadc_mean = (st.mean / 1023.0f) * vref;
  const float vadc_unc = (st.stddev / 1023.0f) * vref;
  const bool calibrated = gCalValid;
  const float K = calibrated ? gCal.K : 1.0f;
  const float vin_mean = vadc_mean * K;
  const float vin_unc = vadc_unc * K;
  gIIRVin = (IIR_ALPHA * gIIRVin) + ((1.0f - IIR_ALPHA) * vin_mean);

  Serial.print(F("t=")); Serial.print(t); Serial.print(F(" ms, ADC_raw_mean=")); Serial.print(st.mean, 2);
  Serial.print(F(" (min=")); Serial.print(st.rawMin); Serial.print(F(", max=")); Serial.print(st.rawMax); Serial.print(F(")"));
  Serial.print(F(", vref=")); Serial.print(vref, 6); Serial.print(F(", Vadc=")); Serial.print(vadc_mean, 6);
  Serial.print(F(", Vin=")); Serial.print(vin_mean, 6); Serial.print(F(" V"));
  Serial.print(F(" ±")); Serial.print(vin_unc, 6);
  Serial.print(F(", Vin_IIR=")); Serial.print(gIIRVin, 6);
  Serial.print(F(", ref=")); Serial.print(gActiveRef == REF_INTERNAL ? F("INTERNAL") : F("DEFAULT"));
  Serial.print(F(", cal=")); Serial.println(calibrated ? F("YES") : F("NO"));
}

static Stats takeMeasurement(uint16_t samples = 30) { return robustSample(samples); }

// ================== Serial Command Parser ==================
static bool equalsIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a; char cb = *b;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
    ++a; ++b;
  }
  return (*a == 0 && *b == 0);
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F(" CAL <volts> [RBOT <ohms>|RTOP <ohms>] [SAMPLES <n>]"));
  Serial.println(F(" PRINTCAL"));
  Serial.println(F(" RESETCAL | CLEARCAL"));
  Serial.println(F(" SETREF DEFAULT|INTERNAL|AUTO"));
  Serial.println(F(" MEASURE"));
  Serial.println(F(" LOG ON|OFF"));
}

static void printCalSummary() {
  if (!gCalValid) { Serial.println(F("[CAL] No valid calibration in EEPROM.")); return; }
  Serial.println(F("[CAL] Current calibration:"));
  Serial.print(F(" Ref : ")); Serial.println(gCal.adcRef == REF_INTERNAL ? F("INTERNAL 1.1V") : F("DEFAULT Vcc"));
  Serial.print(F(" VrefUsed : ")); Serial.println(gCal.vrefUsed, 6);
  Serial.print(F(" Samples : ")); Serial.println(gCal.sampleCount);
  Serial.print(F(" ADC Mean : ")); Serial.println(gCal.adcMean, 4);
  Serial.print(F(" ADC Stddev: ")); Serial.println(gCal.adcStddev, 4);
  Serial.print(F(" Ratio K : ")); Serial.println(gCal.K, 8);
  Serial.print(F(" RTOP est : ")); Serial.println(gCal.rTop, 2);
  Serial.print(F(" RBOT est : ")); Serial.println(gCal.rBot, 2);
  Serial.print(F(" CalMillis : ")); Serial.println(gCal.calMillis);
  Serial.print(F(" CalCounter: ")); Serial.println(gCal.calCounter);
  Serial.print(F(" CRC16 : 0x")); Serial.println(gCal.crc16, HEX);
}

static void handleSetRef(const char* arg) {
  if (equalsIgnoreCase(arg, "DEFAULT")) { gRefMode = REF_DEFAULT; applyReference(REF_DEFAULT); Serial.println(F("[SETREF] DEFAULT")); }
  else if (equalsIgnoreCase(arg, "INTERNAL")) { gRefMode = REF_INTERNAL; applyReference(REF_INTERNAL); Serial.println(F("[SETREF] INTERNAL")); }
  else if (equalsIgnoreCase(arg, "AUTO")) { gRefMode = REF_AUTO; selectReferenceAuto(); Serial.println(F("[SETREF] AUTO")); }
  else Serial.println(F("[SETREF] Invalid argument."));
}

static void runCalibration(float vinKnown, uint16_t samples, bool hasRbot, float rbotVal, bool hasRtop, float rtopVal) {
  if (gRefMode == REF_AUTO) selectReferenceAuto(); else applyReference(gRefMode);
  Serial.print(F("[CAL] Vin_known=")); Serial.print(vinKnown, 6); Serial.print(F(", samples=")); Serial.println(samples);

  Stats st = robustSample(samples);
  float vref = currentVrefVolts();
  float vadc = (st.mean / 1023.0f) * vref;
  float K = vadc > 0.0f ? (vinKnown / vadc) : 1.0f;

  float estRTOP = 0.0f, estRBOT = 0.0f;
  if (hasRbot && rbotVal > 0.0f) { estRBOT = rbotVal; estRTOP = rbotVal * (K - 1.0f); }
  else if (hasRtop && rtopVal > 0.0f) { estRTOP = rtopVal; estRBOT = rtopVal / (K - 1.0f); }

  CalData cd;
  cd.magic = CAL_MAGIC; cd.version = CAL_VERSION; cd.adcRef = gActiveRef;
  cd.sampleCount = samples; cd.vrefUsed = vref; cd.K = K; cd.rTop = estRTOP; cd.rBot = estRBOT;
  cd.adcMean = st.mean; cd.adcStddev = st.stddev; cd.calMillis = millis();
  cd.calCounter = gCalValid ? (gCal.calCounter + 1) : 1; cd.crc16 = 0;

  Serial.print(F("[CAL] ADC mean=")); Serial.print(st.mean,4); Serial.print(F(", stddev=")); Serial.println(st.stddev,4);

  if (eepromSaveCalibration(cd)) { gCal = cd; gCalValid = true; Serial.println(F("[CAL] Saved.")); } else Serial.println(F("[CAL] Save failed."));
}

static void handleClearCal() { eepromClearCalibration(); Serial.println(F("[CAL] Cleared.")); }
static void handleMeasureOnce() { if (gRefMode == REF_AUTO) selectReferenceAuto(); else applyReference(gRefMode); Stats st = takeMeasurement(30); printMeasurementLine(st); }
static void handleLog(const char* arg) { if (equalsIgnoreCase(arg,"ON")) { gLoggingEnabled = true; Serial.println(F("[LOG] ON")); } else if (equalsIgnoreCase(arg,"OFF")) { gLoggingEnabled = false; Serial.println(F("[LOG] OFF")); } else Serial.println(F("[LOG] Invalid.")); }

static void serialCommandHandler(char* line) {
  const char* delims = " ,;\t";
  char* cmd = strtok(line, delims);
  if (!cmd) return;
  if (equalsIgnoreCase(cmd, "HELP")) { printHelp(); return; }
  if (equalsIgnoreCase(cmd, "PRINTCAL")) { printCalSummary(); return; }
  if (equalsIgnoreCase(cmd, "RESETCAL") || equalsIgnoreCase(cmd, "CLEARCAL")) { handleClearCal(); return; }
  if (equalsIgnoreCase(cmd, "SETREF")) { char* arg = strtok(NULL, delims); if (arg) handleSetRef(arg); else Serial.println(F("[SETREF] Missing arg.")); return; }
  if (equalsIgnoreCase(cmd, "MEASURE")) { handleMeasureOnce(); return; }
  if (equalsIgnoreCase(cmd, "LOG")) { char* arg = strtok(NULL, delims); if (arg) handleLog(arg); else Serial.println(F("[LOG] Missing arg.")); return; }
  if (equalsIgnoreCase(cmd, "CAL")) {
    char* vstr = strtok(NULL, delims);
    if (!vstr) { Serial.println(F("[CAL] Usage: CAL <volts> [RBOT <ohms>|RTOP <ohms>] [SAMPLES <n>]")); return; }
    float vinKnown = atof(vstr);
    bool hasRbot=false, hasRtop=false; float rbotVal=0.0f, rtopVal=0.0f; uint16_t samples=DEFAULT_CAL_SAMPLES;
    char* tok = strtok(NULL, delims);
    while (tok) {
      if (equalsIgnoreCase(tok, "RBOT")) { char* r=strtok(NULL,delims); if (r) { hasRbot=true; rbotVal = atof(r); } }
      else if (equalsIgnoreCase(tok, "RTOP")) { char* r=strtok(NULL,delims); if (r) { hasRtop=true; rtopVal = atof(r); } }
      else if (equalsIgnoreCase(tok, "SAMPLES")) { char* s=strtok(NULL,delims); if (s) { long n=atol(s); if (n>0) samples=(uint16_t)n; } }
      tok = strtok(NULL, delims);
    }
    runCalibration(vinKnown, samples, hasRbot, rbotVal, hasRtop, rtopVal);
    return;
  }
  Serial.println(F("[CMD] Unknown. Type HELP."));
}

// ================== Setup & Loop ==================
static void processSerial() {
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (idx > 0) { gSerialBuf[idx] = 0; serialCommandHandler(gSerialBuf); }
      idx = 0;
    } else {
      if (idx < (int)sizeof(gSerialBuf) - 1) gSerialBuf[idx++] = c;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(ANALOG_PIN, INPUT);
  eepromReadBootCounter();
  eepromIncrementBootCounter();
  gCalValid = eepromLoadCalibration();
  if (gRefMode == REF_AUTO) selectReferenceAuto(); else applyReference(gRefMode);
  printStartupReport();
  Serial.println(F("Type HELP for commands."));
}

void loop() {
  processSerial();
  if (gLoggingEnabled) {
    uint32_t now = millis();
    if (now - gLastLogMs >= gLogIntervalMs) {
      if (gRefMode == REF_AUTO) selectReferenceAuto(); else applyReference(gRefMode);
      Stats st = takeMeasurement(30);
      printMeasurementLine(st);
      gLastLogMs = now;
    }
  }
}
