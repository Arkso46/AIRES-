#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// WiFi + Web server
#include <WiFi.h>
#include <WebServer.h>

// Sensirion headers
#include <SensirionI2cScd4x.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2cSps30.h>

#include <Adafruit_BMP280.h>

// ---------------------------------------------------------------------------
// PIN CONFIG
// ---------------------------------------------------------------------------

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

const int FAN_PWM_PIN     = 25;
const int FAN_PWM_CHANNEL = 0;
const int FAN_PWM_FREQ    = 25000; // 25 kHz
const int FAN_PWM_RES     = 8;     // 8-bit

// ---------------------------------------------------------------------------
// WIFI CONFIG
// ---------------------------------------------------------------------------

const char* WIFI_SSID     = "Spensas S25 Edge";
const char* WIFI_PASSWORD = "coolpanda687";

WebServer server(80);

// ---------------------------------------------------------------------------
// GLOBALS – SENSORS
// ---------------------------------------------------------------------------

SensirionI2cScd4x scd4x;   // SCD41 CO2 + T + RH
SensirionI2CSgp41 sgp41;   // SGP41 gas sensor
SensirionI2cSps30 sps30;   // SPS30 particulate sensor
Adafruit_BMP280   bmp;     // BMP280 env sensor (pressure + backup temp)

static unsigned long g_LastPrintMs    = 0;
const unsigned long PRINT_INTERVAL_MS = 5000;   // every 5 seconds

// Fan duty tracking (for logs + web)
static int g_LastFanDuty = -1;

// Env / BMP / SCD
float g_TempC      = NAN;   // unified temp used everywhere (SCD41 preferred)
float g_TempF      = NAN;
float g_PressurePa = NAN;
float g_Rh         = NAN;   // %RH from SCD41 when available
bool  g_BmpOk      = false;

// SPS30 – PM2.5
float g_PM25         = NAN;   // raw
float g_PM25Filtered = NAN;   // filtered
const float PM_FILTER_ALPHA = 0.2f;

int  g_Sps30ErrorCount     = 0;
const int SPS30_MAX_ERRORS = 5;
bool g_Sps30Fault          = false;

// SGP41
bool     g_Sgp41Fault                  = false;
int      g_Sgp41ErrorCount             = 0;
const int SGP41_MAX_ERRORS             = 5;
uint16_t g_Sgp41ConditioningSeconds    = 10;

uint16_t g_SgpSrawVoc = 0;
uint16_t g_SgpSrawNox = 0;

int32_t  g_VocIndex    = -1;
int32_t  g_NoxIndex    = -1;

float    g_VocBaseline = NAN;
float    g_NoxBaseline = NAN;

static unsigned long g_LastSgp41Ms     = 0;
const unsigned long  SGP41_INTERVAL_MS = 1000;

const uint16_t SGP41_DEFAULT_RH = 0x8000;
const uint16_t SGP41_DEFAULT_T  = 0x6666;

// SCD41
bool     g_Scd41Fault        = false;
int      g_Scd41ErrorCount   = 0;
const int SCD41_MAX_ERRORS   = 5;
float    g_Co2Ppm            = NAN;
float    g_ScdTempC          = NAN;
float    g_ScdRh             = NAN;
unsigned long g_LastScd41Ms  = 0;
const unsigned long SCD41_INTERVAL_MS = 5000;

// ---------------------------------------------------------------------------
// FAN OVERRIDE STATE (for web controls)
// ---------------------------------------------------------------------------

bool g_FanManualOverride = false; // false = auto, true = manual
int  g_FanManualDuty     = 20;    // 0–100, in 25% steps via UI

// ---------------------------------------------------------------------------
// WEB DASHBOARD STATE
// ---------------------------------------------------------------------------

String g_statusPM   = "n/a";
String g_statusVOC  = "n/a";
String g_statusNOx  = "n/a";
String g_statusTemp = "n/a";
String g_statusCO2  = "n/a";

String g_hazardText  = "unknown";
String g_hazardLevel = "normal";  // "normal", "warning", "dangerous"

// ---------------------------------------------------------------------------
// I2C SCAN
// ---------------------------------------------------------------------------

void scanI2C() {
    Serial.println("=== I2C SCAN ===");

    byte count = 0;
    for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("  Found: 0x");
            Serial.println(addr, HEX);
            count++;
        }
    }
    if (count == 0) Serial.println("  No I2C devices found.");
    Serial.println();
}

// ---------------------------------------------------------------------------
// °C → SGP41 TEMP TICKS
// ---------------------------------------------------------------------------

uint16_t tempCToSgpTicks(float tempC) {
    if (isnan(tempC)) {
        return SGP41_DEFAULT_T;
    }
    float ticks = ((tempC + 45.0f) * 65535.0f) / 175.0f;
    if (ticks < 0.0f)     ticks = 0.0f;
    if (ticks > 65535.0f) ticks = 65535.0f;
    return (uint16_t)ticks;
}

// ---------------------------------------------------------------------------
// SCD41 INIT + PERIODIC READ
// ---------------------------------------------------------------------------

void initScd41() {
    Serial.println("Init SCD41...");

    scd4x.begin(Wire, 0x62); // default I2C address

    uint16_t error;

    scd4x.stopPeriodicMeasurement();
    delay(500);

    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("  SCD41 startPeriodicMeasurement failed, error = ");
        Serial.println(error);
        g_Scd41Fault      = true;
        g_Scd41ErrorCount = SCD41_MAX_ERRORS;
    } else {
        Serial.println("  SCD41 measurement started");
        g_Scd41Fault      = false;
        g_Scd41ErrorCount = 0;
    }

    g_Co2Ppm   = NAN;
    g_ScdTempC = NAN;
    g_ScdRh    = NAN;
}

void readScd41Periodic() {
    if (g_Scd41Fault) return;

    unsigned long now = millis();
    if (now - g_LastScd41Ms < SCD41_INTERVAL_MS) return;
    g_LastScd41Ms = now;

    bool dataReady = false;
    uint16_t error = scd4x.getDataReadyStatus(dataReady);
    if (error) {
        Serial.print("[SCD41] getDataReadyStatus error = ");
        Serial.println(error);
        g_Scd41ErrorCount++;
        if (g_Scd41ErrorCount >= SCD41_MAX_ERRORS) {
            g_Scd41Fault = true;
            Serial.println("[SCD41] Fault: too many consecutive errors");
        }
        return;
    }

    if (!dataReady) return;

    uint16_t co2 = 0;
    float tempC = NAN;
    float rh = NAN;

    error = scd4x.readMeasurement(co2, tempC, rh);
    if (error) {
        Serial.print("[SCD41] readMeasurement error = ");
        Serial.println(error);
        g_Scd41ErrorCount++;
        if (g_Scd41ErrorCount >= SCD41_MAX_ERRORS) {
            g_Scd41Fault = true;
            Serial.println("[SCD41] Fault: too many consecutive errors");
        }
        return;
    }

    g_Scd41ErrorCount = 0;
    g_Scd41Fault      = false;

    g_Co2Ppm   = (float)co2;
    g_ScdTempC = tempC;
    g_ScdRh    = rh;

    Serial.print("[SCD41] CO2=");
    Serial.print(g_Co2Ppm, 1);
    Serial.print(" ppm  T=");
    Serial.print(g_ScdTempC, 2);
    Serial.print(" °C  RH=");
    Serial.print(g_ScdRh, 1);
    Serial.println(" %");
}

// ---------------------------------------------------------------------------
// SGP41 INIT
// ---------------------------------------------------------------------------

void initSgp41() {
    Serial.println("Init SGP41...");

    sgp41.begin(Wire);

    uint16_t error;
    uint16_t testResult = 0;

    error = sgp41.executeSelfTest(testResult);
    if (error) {
        Serial.print("  SGP41 executeSelfTest error = ");
        Serial.println(error);
        g_Sgp41Fault = true;
    } else if (testResult != 0xD400) {
        Serial.print("  SGP41 self-test failed, result = 0x");
        Serial.println(testResult, HEX);
        g_Sgp41Fault = true;
    } else {
        Serial.println("  SGP41 self-test OK");
        g_Sgp41Fault      = false;
        g_Sgp41ErrorCount = 0;
    }

    g_Sgp41ConditioningSeconds = 10;
    g_VocBaseline = NAN;
    g_NoxBaseline = NAN;
    g_VocIndex    = -1;
    g_NoxIndex    = -1;
}

// ---------------------------------------------------------------------------
// SENSOR INIT
// ---------------------------------------------------------------------------

void initSensors() {
    Serial.println("=== SENSOR INIT ===");

    initScd41();
    initSgp41();

    sps30.begin(Wire, SPS30_I2C_ADDR_69);
    sps30.stopMeasurement();
    int16_t error = sps30.startMeasurement((SPS30OutputFormat)1280);
    if (error) {
        Serial.print("  SPS30 startMeasurement failed, error = ");
        Serial.println(error);
        g_Sps30Fault = true;
    } else {
        Serial.println("  SPS30 measurement started");
        g_Sps30Fault      = false;
        g_Sps30ErrorCount = 0;
    }

    g_BmpOk = bmp.begin(0x76);
    if (!g_BmpOk) {
        g_BmpOk = bmp.begin(0x77);
    }
    if (!g_BmpOk) {
        Serial.println("  BMP280 NOT found!");
    } else {
        Serial.println("  BMP280 OK");
    }

    Serial.println("Sensor init done.\n");
}

// ---------------------------------------------------------------------------
// FAN CONTROL
// ---------------------------------------------------------------------------

void setFanDutyPercent(int dutyPercent) {
    if (dutyPercent < 0)   dutyPercent = 0;
    if (dutyPercent > 100) dutyPercent = 100;

    if (dutyPercent == g_LastFanDuty) return;

    g_LastFanDuty = dutyPercent;

    int pwmValue = map(dutyPercent, 0, 100, 0, 255);
    ledcWrite(FAN_PWM_CHANNEL, pwmValue);

    Serial.print("[FAN] Duty changed to ");
    Serial.print(dutyPercent);
    Serial.println("%");
}

// ---------------------------------------------------------------------------
// SPS30 – READ + FILTER
// ---------------------------------------------------------------------------

float readPM25FromSps30() {
    if (g_Sps30Fault) return g_PM25;

    uint16_t dataReadyFlag = 0;
    int16_t error = sps30.readDataReadyFlag(dataReadyFlag);
    if (error) {
        Serial.print("[SPS30] readDataReadyFlag error = ");
        Serial.println(error);
        g_Sps30ErrorCount++;
        if (g_Sps30ErrorCount >= SPS30_MAX_ERRORS) {
            g_Sps30Fault = true;
            Serial.println("[SPS30] Fault: too many consecutive errors");
        }
        return g_PM25;
    }

    if (!dataReadyFlag) return g_PM25;

    uint16_t mc1p0 = 0, mc2p5 = 0, mc4p0 = 0, mc10p0 = 0;
    uint16_t nc0p5 = 0, nc1p0 = 0, nc2p5 = 0, nc4p0 = 0, nc10p0 = 0;
    uint16_t typicalParticleSize = 0;

    error = sps30.readMeasurementValuesUint16(
        mc1p0, mc2p5, mc4p0, mc10p0,
        nc0p5, nc1p0, nc2p5, nc4p0,
        nc10p0, typicalParticleSize
    );
    if (error) {
        Serial.print("[SPS30] readMeasurementValuesUint16 error = ");
        Serial.println(error);
        g_Sps30ErrorCount++;
        if (g_Sps30ErrorCount >= SPS30_MAX_ERRORS) {
            g_Sps30Fault = true;
            Serial.println("[SPS30] Fault: too many consecutive errors");
        }
        return g_PM25;
    }

    g_Sps30ErrorCount = 0;
    g_Sps30Fault      = false;

    g_PM25 = mc2p5 / 10.0f;
    return g_PM25;
}

// ---------------------------------------------------------------------------
// SIMPLE INDEX HELPER
// ---------------------------------------------------------------------------

int32_t computeIndexFromBaseline(float baseline, float current, float scale, bool invert) {
    if (isnan(baseline) || scale <= 0.0f) {
        return -1;
    }

    float delta = invert ? (baseline - current) : (current - baseline);
    float idx = 100.0f + (delta / scale);
    if (idx < 0.0f)   idx = 0.0f;
    if (idx > 500.0f) idx = 500.0f;

    return (int32_t)(idx + 0.5f);
}

// ---------------------------------------------------------------------------
// SGP41 PERIODIC READ
// ---------------------------------------------------------------------------

void readSgp41Periodic() {
    if (g_Sgp41Fault) return;

    unsigned long now = millis();
    if (now - g_LastSgp41Ms < SGP41_INTERVAL_MS) return;
    g_LastSgp41Ms = now;

    uint16_t error;
    uint16_t srawVoc = 0;
    uint16_t srawNox = 0;

    uint16_t tTicks = tempCToSgpTicks(g_TempC);
    uint16_t rhTicks = SGP41_DEFAULT_RH;

    if (g_Sgp41ConditioningSeconds > 0) {
        error = sgp41.executeConditioning(rhTicks, tTicks, srawVoc);
        g_Sgp41ConditioningSeconds--;
    } else {
        error = sgp41.measureRawSignals(rhTicks, tTicks, srawVoc, srawNox);
    }

    if (error) {
        Serial.print("[SGP41] measure error = ");
        Serial.println(error);
        g_Sgp41ErrorCount++;
        if (g_Sgp41ErrorCount >= SGP41_MAX_ERRORS) {
            g_Sgp41Fault = true;
            Serial.println("[SGP41] Fault: too many consecutive errors");
        }
        return;
    }

    g_Sgp41ErrorCount = 0;
    g_Sgp41Fault      = false;

    g_SgpSrawVoc = srawVoc;
    g_SgpSrawNox = srawNox;

    if (g_Sgp41ConditioningSeconds == 0) {
        if (isnan(g_VocBaseline)) {
            g_VocBaseline = (float)g_SgpSrawVoc;
        } else {
            g_VocBaseline = 0.99f * g_VocBaseline + 0.01f * (float)g_SgpSrawVoc;
        }

        if (g_SgpSrawNox != 0) {
            if (isnan(g_NoxBaseline)) {
                g_NoxBaseline = (float)g_SgpSrawNox;
            } else {
                g_NoxBaseline = 0.99f * g_NoxBaseline + 0.01f * (float)g_SgpSrawNox;
            }
        }

        g_VocIndex = computeIndexFromBaseline(
            g_VocBaseline,
            (float)g_SgpSrawVoc,
            50.0f,
            true
        );

        g_NoxIndex = computeIndexFromBaseline(
            g_NoxBaseline,
            (float)g_SgpSrawNox,
            80.0f,
            false
        );
    } else {
        g_VocIndex = -1;
        g_NoxIndex = -1;
    }

    Serial.print("[SGP41] raw @1Hz  VOC=");
    Serial.print(srawVoc);
    Serial.print("  NOx=");
    Serial.println(srawNox);
}

// ---------------------------------------------------------------------------
// PERIODIC SENSOR READS
// ---------------------------------------------------------------------------

void readSensorsPeriodic() {
    // SCD41 first (CO2 + T + RH)
    readScd41Periodic();

    // PM
    readPM25FromSps30();
    if (!isnan(g_PM25)) {
        if (isnan(g_PM25Filtered)) {
            g_PM25Filtered = g_PM25;
        } else {
            g_PM25Filtered = PM_FILTER_ALPHA * g_PM25 +
                             (1.0f - PM_FILTER_ALPHA) * g_PM25Filtered;
        }
    }

    // Env: prefer SCD41 temp+RH, BMP temp as backup, BMP for pressure
    if (!g_Scd41Fault && !isnan(g_ScdTempC)) {
        g_TempC = g_ScdTempC;
        g_TempF = g_TempC * 9.0f / 5.0f + 32.0f;
        g_Rh    = g_ScdRh;
    } else if (g_BmpOk) {
        float bmpTemp = bmp.readTemperature();
        g_TempC = bmpTemp;
        g_TempF = g_TempC * 9.0f / 5.0f + 32.0f;
        g_Rh    = NAN;
    } else {
        g_TempC = NAN;
        g_TempF = NAN;
        g_Rh    = NAN;
    }

    if (g_BmpOk) {
        g_PressurePa = bmp.readPressure();
    } else {
        g_PressurePa = NAN;
    }

    // Gas
    readSgp41Periodic();
}

// ---------------------------------------------------------------------------
// LABEL HELPERS
// ---------------------------------------------------------------------------

const char* pm25Label(float pm25) {
    if (isnan(pm25)) return "n/a";
    if (pm25 < 12.0f)  return "normal";
    if (pm25 < 35.0f)  return "medium";
    if (pm25 < 55.0f)  return "high";
    return "dangerous";
}

const char* vocLabelFromIndex(int32_t idx) {
    if (idx < 0)   return "n/a";
    if (idx < 140) return "normal";
    if (idx < 190) return "medium";
    if (idx < 240) return "high";
    return "dangerous";
}

const char* noxLabelFromIndex(int32_t idx) {
    if (idx < 0)   return "n/a";
    if (idx < 140) return "normal";
    if (idx < 200) return "medium";
    if (idx < 280) return "high";
    return "dangerous";
}

const char* tempLabel(float tC) {
    if (isnan(tC))   return "n/a";
    if (tC < 5.0f)   return "dangerous (cold)";
    if (tC < 12.0f)  return "high (cold)";
    if (tC < 18.0f)  return "medium (cool)";
    if (tC < 27.0f)  return "normal";
    if (tC < 32.0f)  return "medium (warm)";
    if (tC < 37.0f)  return "high (hot)";
    return "dangerous (hot)";
}

// SOFTER CO₂ LABELS
const char* co2Label(float co2ppm) {
    if (isnan(co2ppm)) return "n/a";
    if (co2ppm < 1200.0f)   return "normal";
    if (co2ppm < 2000.0f)   return "medium";
    if (co2ppm < 3000.0f)   return "high";
    return "dangerous";
}

// ---------------------------------------------------------------------------
// SEVERITY HELPERS
// ---------------------------------------------------------------------------

int pmSeverity(float pm25) {
    if (isnan(pm25)) return 0;
    if (pm25 < 12.0f)  return 0;
    if (pm25 < 35.0f)  return 1;
    if (pm25 < 55.0f)  return 2;
    return 3;
}

int vocSeverity(int32_t idx) {
    if (idx < 0)   return 0;
    if (idx < 140) return 0;
    if (idx < 190) return 1;
    if (idx < 240) return 2;
    return 3;
}

int noxSeverity(int32_t idx) {
    if (idx < 0)   return 0;
    if (idx < 140) return 0;
    if (idx < 200) return 1;
    if (idx < 280) return 2;
    return 3;
}

int tempSeverity(float tC) {
    if (isnan(tC)) return 0;
    if (tC < 5.0f) return 3;
    if (tC < 12.0f) return 2;
    if (tC < 18.0f) return 1;
    if (tC < 27.0f) return 0;
    if (tC < 32.0f) return 1;
    if (tC < 37.0f) return 2;
    return 3;
}

// SOFTER CO₂ SEVERITY
int co2Severity(float co2ppm) {
    if (isnan(co2ppm)) return 0;
    if (co2ppm < 1200.0f)   return 0; // normal
    if (co2ppm < 2000.0f)   return 1; // medium
    if (co2ppm < 3000.0f)   return 2; // high
    return 3;                         // dangerous
}

const char* severityIcon(int sev) {
    switch (sev) {
        case 0: return "-";
        case 1: return "(!)";
        case 2: return "(!!)";
        case 3: return "(!!!)";
    }
    return "-";
}

const char* severityName(int sev) {
    switch (sev) {
        case 0: return "normal";
        case 1: return "medium";
        case 2: return "high";
        case 3: return "dangerous";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// WEB STATUS + HAZARD
// ---------------------------------------------------------------------------

void updateWebStatusAndHazard() {
    g_statusPM   = pm25Label(g_PM25Filtered);
    g_statusVOC  = vocLabelFromIndex(g_VocIndex);
    g_statusNOx  = noxLabelFromIndex(g_NoxIndex);
    g_statusTemp = tempLabel(g_TempC);
    g_statusCO2  = co2Label(g_Co2Ppm);

    int pmSev   = pmSeverity(g_PM25Filtered);
    int vocSev  = vocSeverity(g_VocIndex);
    int noxSev  = noxSeverity(g_NoxIndex);
    int tSev    = tempSeverity(g_TempC);
    int co2Sev  = co2Severity(g_Co2Ppm);

    int   worstSev  = pmSev;
    const char* worstName = "PM";

    if (vocSev > worstSev) {
        worstSev  = vocSev;
        worstName = "VOC";
    }
    if (noxSev > worstSev) {
        worstSev  = noxSev;
        worstName = "NOx";
    }
    if (tSev > worstSev) {
        worstSev  = tSev;
        worstName = "Temp";
    }
    if (co2Sev > worstSev) {
        worstSev  = co2Sev;
        worstName = "CO2";
    }

    g_hazardText = String(worstName) + " – " + severityName(worstSev);

    if (worstSev >= 3) {
        g_hazardLevel = "dangerous";
    } else if (worstSev >= 1) {
        g_hazardLevel = "warning";
    } else {
        g_hazardLevel = "normal";
    }
}

// ---------------------------------------------------------------------------
// FAN LOGIC (auto + manual override)
// ---------------------------------------------------------------------------

void updateFanFromAirQuality() {
    if (g_FanManualOverride) {
        setFanDutyPercent(g_FanManualDuty);
        return;
    }

    if (g_Sps30Fault) {
        setFanDutyPercent(80);
        return;
    }

    float pm25 = !isnan(g_PM25Filtered) ? g_PM25Filtered : g_PM25;
    int duty = 20;

    if (isnan(pm25)) {
        duty = 20;
    } else if (pm25 < 12.0f) {
        duty = 20;
    } else if (pm25 < 35.0f) {
        duty = 40;
    } else if (pm25 < 55.0f) {
        duty = 70;
    } else {
        duty = 100;
    }

    if (g_VocIndex >= 0) {
        if (g_VocIndex >= 240) {
            if (duty < 100) duty = 100;
        } else if (g_VocIndex >= 190) {
            if (duty < 80) duty = 80;
        } else if (g_VocIndex >= 140) {
            if (duty < 60) duty = 60;
        }
    }

    setFanDutyPercent(duty);
}

// ---------------------------------------------------------------------------
// STATUS PRINT (Serial AIR SUMMARY)
// ---------------------------------------------------------------------------

void printStatus() {
    const char* pmLabel  = pm25Label(g_PM25Filtered);
    const char* vocLabel = vocLabelFromIndex(g_VocIndex);
    const char* noxLabel = noxLabelFromIndex(g_NoxIndex);
    const char* tLabel   = tempLabel(g_TempC);
    const char* co2Lbl   = co2Label(g_Co2Ppm);

    int pmSev   = pmSeverity(g_PM25Filtered);
    int vocSev  = vocSeverity(g_VocIndex);
    int noxSev  = noxSeverity(g_NoxIndex);
    int tSev    = tempSeverity(g_TempC);
    int co2Sev  = co2Severity(g_Co2Ppm);

    int   worstSev  = pmSev;
    const char* worstName = "PM";

    if (vocSev > worstSev) {
        worstSev  = vocSev;
        worstName = "VOC";
    }
    if (noxSev > worstSev) {
        worstSev  = noxSev;
        worstName = "NOx";
    }
    if (tSev > worstSev) {
        worstSev  = tSev;
        worstName = "Temp";
    }
    if (co2Sev > worstSev) {
        worstSev  = co2Sev;
        worstName = "CO2";
    }

    Serial.println("\n===== AIRES STATUS =====");

    Serial.print("AIR SUMMARY: ");
    Serial.print("PM ");
    Serial.print(pmLabel);
    Serial.print(" | VOC ");
    Serial.print(vocLabel);
    Serial.print(" | NOx ");
    Serial.print(noxLabel);
    Serial.print(" | Temp ");
    Serial.print(tLabel);
    Serial.print(" | CO2 ");
    Serial.println(co2Lbl);

    Serial.print("HAZARD PRIORITY: ");
    Serial.print(severityIcon(worstSev));
    Serial.print(" ");
    Serial.print(worstName);
    Serial.print(" – ");
    Serial.println(severityName(worstSev));

    Serial.println("\n[FAN]");
    Serial.print("  Duty ................. ");
    Serial.print(g_LastFanDuty);
    Serial.println(" %");

    Serial.println("\n[PM]");
    Serial.print("  Status ............... ");
    Serial.println(g_Sps30Fault ? "FAULT" : "OK");
    Serial.print("  PM2.5 (raw) .......... ");
    Serial.print(g_PM25);
    Serial.println(" µg/m³");
    Serial.print("  PM2.5 (filtered) ..... ");
    Serial.print(g_PM25Filtered);
    Serial.print(" µg/m³   (");
    Serial.print(pmLabel);
    Serial.println(")");

    Serial.println("\n[ENV]");
    Serial.print("  Temp ................. ");
    Serial.print(g_TempC, 1);
    Serial.print(" °C / ");
    Serial.print(g_TempF, 1);
    Serial.print(" °F   (");
    Serial.print(tLabel);
    Serial.println(")");
    Serial.print("  RH ................... ");
    if (isnan(g_Rh)) {
        Serial.println("n/a");
    } else {
        Serial.print(g_Rh, 1);
        Serial.println(" %");
    }
    Serial.print("  Pressure ............. ");
    Serial.print(g_PressurePa / 100.0f, 1);
    Serial.println(" hPa");

    Serial.println("\n[CO2]");
    Serial.print("  CO2 .................. ");
    if (isnan(g_Co2Ppm)) {
        Serial.println("n/a");
    } else {
        Serial.print(g_Co2Ppm, 1);
        Serial.println(" ppm");
    }

    Serial.println("\n[GAS]");
    Serial.print("  Status ............... ");
    Serial.println(g_Sgp41Fault ? "FAULT" : "OK");
    Serial.print("  VOC Raw .............. ");
    Serial.println(g_SgpSrawVoc);
    Serial.print("  VOC Index ............ ");
    Serial.print(g_VocIndex);
    Serial.print("   (");
    Serial.print(vocLabel);
    Serial.println(")");
    Serial.print("  NOx Raw .............. ");
    Serial.println(g_SgpSrawNox);
    Serial.print("  NOx Index ............ ");
    Serial.print(g_NoxIndex);
    Serial.print("   (");
    Serial.print(noxLabel);
    Serial.println(")");

    Serial.println("========================\n");
}

// ---------------------------------------------------------------------------
// WIFI + WEB SERVER
// ---------------------------------------------------------------------------

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
}

// ---------------------------------------------------------------------------
// HTML DASHBOARD (VS CODE red theme + CO2 chart)
// ---------------------------------------------------------------------------

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <title>AIRES Live Dashboard</title>
  <style>
    :root {
      --bg-main: #190005;
      --bg-panel: #2b050d;
      --bg-panel-alt: #3a0712;
      --border-subtle: #4b0c18;
      --accent-blue: #a8d8ff;
      --accent-blue-soft: rgba(168,216,255,0.25);
      --accent-pink: #ff8ba7;
      --text-main: #fef3f5;
      --text-muted: #c9a9b4;
      --chip-bg: #3d0b16;
      --chip-bg-active: #5b1021;
    }

    body {
      font-family: system-ui, sans-serif;
      margin:16px;
      background:
        radial-gradient(circle at top left, #3b0916 0, #190005 45%, #070006 100%);
      color:var(--text-main);
    }

    h1 { margin-bottom:4px; }

    .label {
      font-size:0.8rem;
      text-transform:uppercase;
      opacity:0.8;
      color:var(--text-muted);
    }
    .value {
      font-size:1.1rem;
      font-weight:600;
      margin-top:4px;
      color:var(--text-main);
    }

    .cards {
      display:flex;
      flex-wrap:wrap;
      gap:12px;
      margin-bottom:12px;
    }
    .card {
      background:linear-gradient(145deg,var(--bg-panel),var(--bg-panel-alt));
      border-radius:10px;
      padding:10px 12px;
      min-width:220px;
      box-shadow:0 6px 14px rgba(0,0,0,0.55);
      border:1px solid var(--border-subtle);
    }

    #hazardBox {
      border-radius:10px;
      padding:10px 12px;
      margin-top:8px;
      transition:background 0.2s;
    }
    #hazardBox.normal    { background:#12331f; }
    #hazardBox.warning   { background:#3b2a1f; }
    #hazardBox.dangerous { background:#4b1515; }

    .controls {
      display:flex;
      flex-wrap:wrap;
      gap:8px;
      align-items:center;
      margin:12px 0;
    }
    button {
      border:none;
      border-radius:999px;
      padding:6px 16px;
      background:var(--chip-bg);
      color:var(--text-main);
      font-weight:600;
      cursor:pointer;
      border:1px solid var(--border-subtle);
    }
    button:disabled { opacity:0.4; cursor:default; }
    button.small {
      padding:4px 10px;
      font-size:0.75rem;
      border-radius:999px;
    }
    button.small.active {
      background:var(--chip-bg-active);
      border-color:var(--accent-pink);
    }
    button#btnStart {
      background:var(--chip-bg-active);
    }

    input[type=range] { width:160px; }

    .manual-controls {
      display:flex;
      gap:4px;
      align-items:center;
    }

    .charts {
      display:flex;
      flex-wrap:wrap;
      gap:12px;
      margin-top:4px;
    }
    .chart-card {
      background:linear-gradient(145deg,var(--bg-panel),var(--bg-panel-alt));
      border-radius:10px;
      padding:8px;
      flex:1 1 260px;
      box-shadow:0 6px 14px rgba(0,0,0,0.55);
      border:1px solid var(--border-subtle);
    }
    .chart-card canvas {
      width:100%;
      height:210px;
    }
  </style>
</head>
<body>
  <h1>AIRES Live Dashboard</h1>
  <div class="label" id="connStatus">Connecting...</div>

  <div class="cards">
    <div class="card">
      <div class="label">PM2.5</div>
      <div class="value"><span id="pm25Filtered">-</span> µg/m³</div>
      <div class="label">Status: <span id="pmStatus">-</span></div>
    </div>
    <div class="card">
      <div class="label">VOC Index</div>
      <div class="value"><span id="vocIndex">-</span></div>
      <div class="label">Status: <span id="vocStatus">-</span></div>
    </div>
    <div class="card">
      <div class="label">NOx Index</div>
      <div class="value"><span id="noxIndex">-</span></div>
      <div class="label">Status: <span id="noxStatus">-</span></div>
    </div>
    <div class="card">
      <div class="label">CO₂</div>
      <div class="value"><span id="co2ppm">-</span> ppm</div>
      <div class="label">Status: <span id="co2Status">-</span></div>
    </div>
    <div class="card">
      <div class="label">Environment</div>
      <div class="value">
        <span id="tempC">-</span> °C /
        <span id="tempF">-</span> °F ·
        <span id="rhPercent">-</span> %RH ·
        <span id="pressHpa">-</span> hPa
      </div>
      <div class="label">Status: <span id="tempStatus">-</span></div>
    </div>
    <div class="card">
      <div class="label">Fan Duty</div>
      <div class="value"><span id="fanDuty">0</span> %</div>
      <div class="label" id="fanModeLabel">Fan: AUTO</div>
    </div>
  </div>

  <div id="hazardBox" class="normal">
    <div class="label">Hazard Priority</div>
    <div class="value" id="hazardText">-</div>
  </div>

  <div class="controls">
    <button id="btnStart">Start</button>
    <button id="btnStop" disabled>Stop</button>
    <span class="label">Refresh: <span id="refreshLabel">1.0</span>s</span>
    <input id="refreshSlider" type="range" min="200" max="5000" step="100" value="1000">

    <div class="manual-controls">
      <span class="label">Fan Override:</span>
      <button id="btnFanAuto" class="small">AUTO</button>
      <button class="small" data-duty="0">0%</button>
      <button class="small" data-duty="25">25%</button>
      <button class="small" data-duty="50">50%</button>
      <button class="small" data-duty="75">75%</button>
      <button class="small" data-duty="100">100%</button>
    </div>
  </div>

  <div class="charts">
    <div class="chart-card">
      <div class="label">PM2.5 (µg/m³)</div>
      <canvas id="chartPm"></canvas>
    </div>
    <div class="chart-card">
      <div class="label">VOC &amp; NOx Index</div>
      <canvas id="chartGas"></canvas>
    </div>
    <div class="chart-card">
      <div class="label">CO₂ (ppm)</div>
      <canvas id="chartCo2"></canvas>
    </div>
    <div class="chart-card">
      <div class="label">Env – Temp (°F)</div>
      <canvas id="chartTemp"></canvas>
    </div>
    <div class="chart-card">
      <div class="label">Env – Pressure (hPa)</div>
      <canvas id="chartPress"></canvas>
    </div>
    <div class="chart-card">
      <div class="label">Fan Duty</div>
      <canvas id="chartFan"></canvas>
    </div>
  </div>

  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <script>
    const elConn   = document.getElementById('connStatus');
    const elPm25   = document.getElementById('pm25Filtered');
    const elVoc    = document.getElementById('vocIndex');
    const elNox    = document.getElementById('noxIndex');
    const elTemp   = document.getElementById('tempC');
    const elTempF  = document.getElementById('tempF');
    const elPress  = document.getElementById('pressHpa');
    const elFan    = document.getElementById('fanDuty');
    const elPmSt   = document.getElementById('pmStatus');
    const elVocSt  = document.getElementById('vocStatus');
    const elNoxSt  = document.getElementById('noxStatus');
    const elTempSt = document.getElementById('tempStatus');
    const elHazBox = document.getElementById('hazardBox');
    const elHazTxt = document.getElementById('hazardText');
    const fanModeLabel = document.getElementById('fanModeLabel');
    const elCo2    = document.getElementById('co2ppm');
    const elCo2St  = document.getElementById('co2Status');
    const elRh     = document.getElementById('rhPercent');

    const btnStart = document.getElementById('btnStart');
    const btnStop  = document.getElementById('btnStop');
    const slider   = document.getElementById('refreshSlider');
    const refreshLabel = document.getElementById('refreshLabel');

    const btnFanAuto = document.getElementById('btnFanAuto');
    const manualButtons = document.querySelectorAll('.manual-controls button[data-duty]');

    let refreshMs = parseInt(slider.value);
    refreshLabel.textContent = (refreshMs / 1000).toFixed(1);

    const COLOR_PM    = 'rgba(168,216,255,1)';
    const FILL_PM     = 'rgba(168,216,255,0.18)';
    const COLOR_VOC   = 'rgba(210,160,255,1)';
    const FILL_VOC    = 'rgba(210,160,255,0.20)';
    const COLOR_NOX   = 'rgba(255,183,178,1)';
    const FILL_NOX    = 'rgba(255,183,178,0.20)';
    const COLOR_TEMPF = 'rgba(173,216,230,1)';
    const FILL_TEMPF  = 'rgba(173,216,230,0.20)';
    const COLOR_PRESS = 'rgba(255,204,153,1)';
    const FILL_PRESS  = 'rgba(255,204,153,0.20)';
    const COLOR_FAN   = 'rgba(248,180,217,1)';
    const FILL_FAN    = 'rgba(248,180,217,0.20)';

    const SMOOTH_PM    = 0.25;
    const SMOOTH_GAS   = 0.30;
    const SMOOTH_TEMP  = 0.35;
    const SMOOTH_PRESS = 0.35;
    const SMOOTH_FAN   = 0.40;

    let sPm = null, sVoc = null, sNox = null, sCo2 = null, sTempF = null, sPress = null, sFan = null;

    function blend(prev, next, alpha) {
      if (next == null || isNaN(next)) return prev;
      if (prev == null || isNaN(prev)) return next;
      return prev + alpha * (next - prev);
    }

    function baseLineOptions(titleY, minY, maxY) {
      return {
        animation:false,
        responsive:true,
        scales:{
          x:{ title:{display:true, text:'Time (s)'} },
          y:{
            title:{display:true, text:titleY || ''},
            min: (minY !== null ? minY : undefined),
            max: (maxY !== null ? maxY : undefined),
          }
        },
        plugins:{ legend:{ labels:{ color:'#fef3f5' } } }
      };
    }

    const chartPm = new Chart(document.getElementById('chartPm').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'PM2.5 (µg/m³)',
          data: [],
          borderWidth: 2,
          borderColor: COLOR_PM,
          backgroundColor: FILL_PM,
          tension:0.45,
          cubicInterpolationMode:'monotone',
          pointRadius:1.8,
          pointHoverRadius:3
        }]
      },
      options: baseLineOptions('µg/m³', null, null)
    });

    const chartGas = new Chart(document.getElementById('chartGas').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          {
            label: 'VOC Index',
            data: [],
            borderWidth: 2,
            borderColor: COLOR_VOC,
            backgroundColor: FILL_VOC,
            tension:0.45,
            cubicInterpolationMode:'monotone',
            pointRadius:1.8,
            pointHoverRadius:3
          },
          {
            label: 'NOx Index',
            data: [],
            borderWidth: 2,
            borderColor: COLOR_NOX,
            backgroundColor: FILL_NOX,
            tension:0.45,
            cubicInterpolationMode:'monotone',
            pointRadius:1.8,
            pointHoverRadius:3
          }
        ]
      },
      options: baseLineOptions('Index', 0, null)
    });

    const chartCo2 = new Chart(document.getElementById('chartCo2').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'CO₂ (ppm)',
          data: [],
          borderWidth: 2,
          borderColor: COLOR_VOC,
          backgroundColor: FILL_VOC,
          tension:0.45,
          cubicInterpolationMode:'monotone',
          pointRadius:1.8,
          pointHoverRadius:3
        }]
      },
      options: baseLineOptions('ppm', null, null)
    });

    const chartTemp = new Chart(document.getElementById('chartTemp').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'Temp (°F)',
          data: [],
          borderWidth: 2,
          borderColor: COLOR_TEMPF,
          backgroundColor: FILL_TEMPF,
          tension:0.45,
          cubicInterpolationMode:'monotone',
          pointRadius:1.8,
          pointHoverRadius:3
        }]
      },
      options: baseLineOptions('°F', null, null)
    });

    const chartPress = new Chart(document.getElementById('chartPress').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'Pressure (hPa)',
          data: [],
          borderWidth: 2,
          borderColor: COLOR_PRESS,
          backgroundColor: FILL_PRESS,
          tension:0.45,
          cubicInterpolationMode:'monotone',
          pointRadius:1.8,
          pointHoverRadius:3
        }]
      },
      options: baseLineOptions('hPa', null, null)
    });

    const chartFan = new Chart(document.getElementById('chartFan').getContext('2d'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [{
          label: 'Fan Duty (%)',
          data: [],
          borderWidth: 2,
          borderColor: COLOR_FAN,
          backgroundColor: FILL_FAN,
          tension:0.45,
          cubicInterpolationMode:'monotone',
          pointRadius:1.8,
          pointHoverRadius:3
        }]
      },
      options: baseLineOptions('%', 0, 100)
    });

    const MAX_POINTS = 200;
    let timer = null;

    function pushPoint(chart, t, values) {
      chart.data.labels.push(t.toFixed(1));
      chart.data.datasets.forEach((ds, i) => ds.data.push(values[i]));
      if (chart.data.labels.length > MAX_POINTS) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(ds => ds.data.shift());
      }
      chart.update('none');
    }

    function setFanButtonsActive(duty, mode) {
      manualButtons.forEach(btn => {
        const d = parseInt(btn.dataset.duty);
        btn.classList.toggle('active', mode === 'manual' && d === duty);
      });
      btnFanAuto.classList.toggle('active', mode === 'auto');
    }

    function updateFanModeFromJson(j) {
      const mode = j.mode || j.fan_mode || "auto";
      const duty = j.duty != null ? j.duty :
                   (j.fan_manual_duty != null ? j.fan_manual_duty : 0);

      fanModeLabel.textContent = mode === 'manual'
        ? `Fan: MANUAL (${duty}%)`
        : 'Fan: AUTO';

      setFanButtonsActive(duty, mode);
    }

    async function fetchData() {
      try {
        const r = await fetch('/data');
        const j = await r.json();

        const pmFiltered = j.pm25_filtered;
        const vocIdx     = j.voc_index;
        const noxIdx     = j.nox_index;
        const tempC      = j.temp_c;
        const tempF      = j.temp_f;
        const pressHpa   = j.press_hpa;
        const fanDuty    = j.fan_duty;
        const co2ppm     = j.co2_ppm;
        const rhPercent  = j.rh_percent;

        elConn.textContent = 'Connected';
        elPm25.textContent  = pmFiltered.toFixed(2);
        elVoc.textContent   = vocIdx;
        elNox.textContent   = noxIdx;
        elTemp.textContent  = tempC.toFixed(2);
        elTempF.textContent = tempF.toFixed(2);
        elPress.textContent = pressHpa.toFixed(1);
        elFan.textContent   = fanDuty;

        if (!isNaN(co2ppm)) elCo2.textContent = co2ppm.toFixed(1);
        else                elCo2.textContent = '-';

        if (!isNaN(rhPercent)) elRh.textContent = rhPercent.toFixed(1);
        else                   elRh.textContent = '-';

        elPmSt.textContent   = j.status_pm;
        elVocSt.textContent  = j.status_voc;
        elNoxSt.textContent  = j.status_nox;
        elTempSt.textContent = j.status_temp;
        elCo2St.textContent  = j.status_co2;

        elHazTxt.textContent = j.hazard_text;
        elHazBox.className   = j.hazard_level;

        updateFanModeFromJson({
          mode: j.fan_mode,
          duty: j.fan_manual_duty
        });

        sPm    = blend(sPm,    pmFiltered, SMOOTH_PM);
        sVoc   = blend(sVoc,   vocIdx,     SMOOTH_GAS);
        sNox   = blend(sNox,   noxIdx,     SMOOTH_GAS);
        sCo2   = blend(sCo2,   co2ppm,     SMOOTH_GAS);
        sTempF = blend(sTempF, tempF,      SMOOTH_TEMP);
        sPress = blend(sPress, pressHpa,   SMOOTH_PRESS);
        sFan   = blend(sFan,   fanDuty,    SMOOTH_FAN);

        const t = j.time_s;
        pushPoint(chartPm,   t, [sPm]);
        pushPoint(chartGas,  t, [sVoc, sNox]);
        pushPoint(chartCo2,  t, [sCo2]);
        pushPoint(chartTemp, t, [sTempF]);
        pushPoint(chartPress,t, [sPress]);
        pushPoint(chartFan,  t, [sFan]);
      } catch (e) {
        elConn.textContent = 'Disconnected';
        console.error(e);
      }
    }

    function startPolling() {
      if (timer) return;
      fetchData();
      timer = setInterval(fetchData, refreshMs);
      btnStart.disabled = true;
      btnStop.disabled  = false;
    }

    function stopPolling() {
      if (!timer) return;
      clearInterval(timer);
      timer = null;
      btnStart.disabled = false;
      btnStop.disabled  = true;
    }

    slider.oninput = () => {
      refreshMs = parseInt(slider.value);
      refreshLabel.textContent = (refreshMs / 1000).toFixed(1);
      if (timer) {
        stopPolling();
        startPolling();
      }
    };

    btnStart.onclick = startPolling;
    btnStop.onclick  = stopPolling;

    btnFanAuto.onclick = () => {
      fetch('/fan?mode=auto')
        .then(r => r.json())
        .then(updateFanModeFromJson)
        .catch(console.error);
    };

    manualButtons.forEach(btn => {
      btn.onclick = () => {
        const duty = parseInt(btn.dataset.duty);
        fetch(`/fan?mode=manual&duty=${duty}`)
          .then(r => r.json())
          .then(updateFanModeFromJson)
          .catch(console.error);
      };
    });

    startPolling();
  </script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// HTTP HANDLERS
// ---------------------------------------------------------------------------

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
    static uint32_t t0 = millis();
    float t_s = (millis() - t0) / 1000.0f;

    float press_hpa = isnan(g_PressurePa) ? NAN : (g_PressurePa / 100.0f);

    String json = "{";
    json += "\"time_s\":"        + String(t_s, 2) + ",";
    json += "\"pm25_raw\":"      + String(g_PM25, 2) + ",";
    json += "\"pm25_filtered\":" + String(g_PM25Filtered, 2) + ",";
    json += "\"voc_index\":"     + String(g_VocIndex) + ",";
    json += "\"nox_index\":"     + String(g_NoxIndex) + ",";
    json += "\"temp_c\":"        + String(g_TempC, 2) + ",";
    json += "\"temp_f\":"        + String(g_TempF, 2) + ",";
    json += "\"press_hpa\":"     + String(press_hpa, 1) + ",";
    json += "\"co2_ppm\":"       + String(g_Co2Ppm, 1) + ",";
    json += "\"rh_percent\":"    + String(g_Rh, 1) + ",";
    json += "\"fan_duty\":"      + String(g_LastFanDuty) + ",";
    json += "\"fan_mode\":\""    + String(g_FanManualOverride ? "manual" : "auto") + "\",";
    json += "\"fan_manual_duty\":"+ String(g_FanManualDuty) + ",";
    json += "\"status_pm\":\""   + g_statusPM   + "\",";
    json += "\"status_voc\":\""  + g_statusVOC  + "\",";
    json += "\"status_nox\":\""  + g_statusNOx  + "\",";
    json += "\"status_temp\":\""+ g_statusTemp + "\",";
    json += "\"status_co2\":\"" + g_statusCO2  + "\",";
    json += "\"hazard_text\":\""+ g_hazardText + "\",";
    json += "\"hazard_level\":\""+ g_hazardLevel + "\"";
    json += "}";

    server.send(200, "application/json", json);
}

void handleFan() {
    String mode    = server.hasArg("mode") ? server.arg("mode") : "";
    String dutyStr = server.hasArg("duty") ? server.arg("duty") : "";

    if (mode == "auto") {
        g_FanManualOverride = false;
    } else if (mode == "manual") {
        g_FanManualOverride = true;
        if (dutyStr.length() > 0) {
            int d = dutyStr.toInt();
            if (d < 0)   d = 0;
            if (d > 100) d = 100;
            g_FanManualDuty = d;
            setFanDutyPercent(g_FanManualDuty);
        }
    }

    String resp = "{";
    resp += "\"mode\":\"" + String(g_FanManualOverride ? "manual" : "auto") + "\",";
    resp += "\"duty\":"    + String(g_FanManualOverride ? g_FanManualDuty : g_LastFanDuty);
    resp += "}";

    server.send(200, "application/json", resp);
}

// ---------------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    scanI2C();
    initSensors();

    ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RES);
    ledcAttachPin(FAN_PWM_PIN, FAN_PWM_CHANNEL);

    connectWifi();
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/fan", handleFan);
    server.begin();
    Serial.println("HTTP server started");

    Serial.println("System Ready.\n");
}

void loop() {
    readSensorsPeriodic();
    updateWebStatusAndHazard();
    updateFanFromAirQuality();

    unsigned long now = millis();
    if (now - g_LastPrintMs > PRINT_INTERVAL_MS) {
        printStatus();
        g_LastPrintMs = now;
    }

    server.handleClient();
    delay(10);
}
