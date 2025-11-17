#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>

// --------- CAN & BMS ----------
MCP2515 can0(10);
SoftwareSerial bmsSerial(3,4);  // RX=3, TX=4

// --------- EEPROM ----------
#define EEPROM_MAX_CHARGE    0
#define EEPROM_MAX_DISCHARGE 2

// --------- BMS Requests ----------
byte reqBasic[] = {0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77};
byte reqCell[]  = {0xDD,0xA5,0x04,0x00,0xFF,0xFC,0x77};

// --------- Mess-Variablen ----------
uint16_t packVoltage = 0;        // mV
int16_t  packCurrent = 0;        // mA  (<0 Entladen / >0 Laden)
uint8_t  soc       = 0;          // BMS-SoC (roh)
uint8_t  soc2      = 0;          // endgültiger SoC (kalibriert)
int8_t   bmsTemp   = 0;
int8_t   bminus    = 0;
int8_t   cellTemp  = 0;

uint16_t maxCellVoltage = 0;     // globale Max/Min nur Info
uint16_t minCellVoltage = 0xFFFF;

uint16_t lastMinCellMv = 0;      // aktuelle Zell-Min (mV)
uint16_t lastMaxCellMv = 0;      // aktuelle Zell-Max (mV)

int16_t maxChargeCurrent    = 0;
int16_t maxDischargeCurrent = 0;

uint16_t cycleCount = 0;

volatile bool sendDue = false;
struct can_frame canMsg;         // aus autowp-mcp2515

// ================================================================
// LUT aus deiner Tabelle (1..100 %)
// ================================================================
const uint8_t LUT_N = 100;

const float PROGMEM LUT_SOC[LUT_N] = {
  1,2,3,4,5,6,7,8,9,10,
  11,12,13,14,15,16,17,18,19,20,
  21,22,23,24,25,26,27,28,29,30,
  31,32,33,34,35,36,37,38,39,40,
  41,42,43,44,45,46,47,48,49,50,
  51,52,53,54,55,56,57,58,59,60,
  61,62,63,64,65,66,67,68,69,70,
  71,72,73,74,75,76,77,78,79,80,
  81,82,83,84,85,86,87,88,89,90,
  91,92,93,94,95,96,97,98,99,100
};

const float PROGMEM LUT_VMIN[LUT_N] = {
  2.8675, 2.943625, 3.002375, 3.0495, 3.08725, 3.118125, 3.13675, 3.145625, 3.150875, 3.154625,
  3.1575, 3.16025, 3.162875, 3.166875, 3.1725, 3.17825, 3.182875, 3.187125, 3.190875, 3.19475,
  3.1985, 3.20225, 3.2055, 3.208875, 3.211625, 3.2145, 3.216875, 3.21925, 3.221625, 3.224125,
  3.22675, 3.228625, 3.231, 3.233125, 3.23475, 3.236625, 3.238125, 3.2395, 3.240875, 3.242125,
  3.24325, 3.24425, 3.245125, 3.246125, 3.247, 3.248, 3.248875, 3.249625, 3.250375, 3.25125,
  3.25175, 3.252625, 3.253125, 3.254125, 3.2545, 3.2555, 3.25625, 3.256875, 3.257375, 3.258375,
  3.25875, 3.25975, 3.26075, 3.261625, 3.262625, 3.26375, 3.265375, 3.266875, 3.269, 3.271,
  3.273375, 3.27625, 3.278375, 3.2805, 3.282375, 3.284, 3.28525, 3.286625, 3.28775, 3.2885,
  3.2895, 3.290375, 3.290875, 3.29175, 3.29225, 3.29275, 3.29325, 3.29375, 3.294125, 3.294625,
  3.294625, 3.295125, 3.295375, 3.295625, 3.295625, 3.296, 3.2965, 3.297875, 3.3055, 3.3555
};

const float PROGMEM LUT_VMAX[LUT_N] = {
  2.984875, 3.063125, 3.11925, 3.1645, 3.200375, 3.2305, 3.25225, 3.25925, 3.262125, 3.26475,
  3.26725, 3.270625, 3.273875, 3.278125, 3.283375, 3.2885, 3.294125, 3.299875, 3.305, 3.30975,
  3.314, 3.31775, 3.3215, 3.32525, 3.329, 3.332875, 3.336125, 3.339, 3.341125, 3.34275,
  3.344125, 3.344625, 3.3455, 3.346, 3.3465, 3.347, 3.34725, 3.347375, 3.347875, 3.348375,
  3.348875, 3.349375, 3.34975, 3.35025, 3.35075, 3.35125, 3.351625, 3.352125, 3.352625, 3.353125,
  3.3535, 3.354, 3.355, 3.3555, 3.355875, 3.356875, 3.357375, 3.35825, 3.35925, 3.360125,
  3.361625, 3.3625, 3.363875, 3.365875, 3.36775, 3.37, 3.372375, 3.37475, 3.377625, 3.3795,
  3.380875, 3.381875, 3.38275, 3.38325, 3.38425, 3.384625, 3.385125, 3.386125, 3.3865, 3.3875,
  3.3885, 3.388875, 3.389875, 3.39075, 3.39175, 3.39275, 3.393625, 3.395, 3.3965, 3.397875,
  3.39975, 3.401625, 3.4045, 3.40775, 3.412, 3.417625, 3.426125, 3.44075, 3.4735, 3.579125
};

static inline float pgmF(const float* p){ return pgm_read_float_near(p); }

// ================================================================
// Rolling-Average Puffer Zellspannungen (ca. 1 Minute @ 4 Hz)
// ================================================================
const uint16_t WIN_CELL = 240;          // 60 s bei 4 Hz
uint16_t bufMin[WIN_CELL] = {0};
uint16_t bufMax[WIN_CELL] = {0};
uint32_t sumMin = 0, sumMax = 0;
uint16_t cntCell = 0, idxCell = 0;

// SoC-Fenster für LUT-SoC (ebenfalls ~1 Minute @ 4 Hz)
const uint16_t WIN_SOC = 240;
uint8_t  bufSoc[WIN_SOC] = {0};
uint32_t sumSoc = 0;
uint16_t cntSoc = 0, idxSoc = 0;

int16_t  socShown = -1;   // monotones Halten für LUT-SoC
uint8_t  socLut   = 0;    // geglätteter neuer SoC
unsigned long last250 = 0;

// Lade/Entlade-Erkennung (nur für LUT-Richtung)
bool  charging_mode = true;
const int16_t I_EPS = 0;  // >=0 = Laden, <0 = Entladen

// ------------------------------------------------
// Ruhe-Erkennung & Kalibrier-Logik
// ------------------------------------------------
const int16_t I_REST = 50;                  // |I| < 50 mA => Ruhe
const unsigned long REST_TIME_MS = 300000UL; // 5 Minuten

bool isResting        = false;
bool restCalibrated   = false;
unsigned long restStartMs = 0;

int16_t socOffset = 0;   // wird auf BMS-SoC addiert unter Last
int8_t  lastDelta = 0;   // letzte Korrektur (SOC_LUT - SOC_BMS)

// ================================================================
// Hilfsfunktionen Rolling-Average
// ================================================================
void pushCellMv(uint16_t vminMv, uint16_t vmaxMv){
  if (cntCell < WIN_CELL){
    bufMin[idxCell] = vminMv; sumMin += vminMv;
    bufMax[idxCell] = vmaxMv; sumMax += vmaxMv;
    idxCell++; cntCell++;
  } else {
    sumMin -= bufMin[idxCell];
    sumMax -= bufMax[idxCell];
    bufMin[idxCell] = vminMv; sumMin += vminMv;
    bufMax[idxCell] = vmaxMv; sumMax += vmaxMv;
    idxCell++;
  }
  if (idxCell >= WIN_CELL) idxCell = 0;
}

void pushSoc(uint8_t s){
  if (cntSoc < WIN_SOC){
    bufSoc[idxSoc] = s; sumSoc += s;
    idxSoc++; cntSoc++;
  } else {
    sumSoc -= bufSoc[idxSoc];
    bufSoc[idxSoc] = s; sumSoc += s;
    idxSoc++;
  }
  if (idxSoc >= WIN_SOC) idxSoc = 0;
}

uint8_t lookupSOC_byVoltage(float v, bool charging){
  if (charging){
    // Laden -> obere Kennlinie
    for (uint8_t i=0; i<LUT_N; i++){
      if (v <= pgmF(&LUT_VMAX[i])) return (uint8_t)lroundf(pgmF(&LUT_SOC[i]));
    }
    return 100;
  } else {
    // Entladen -> untere Kennlinie
    for (int i=LUT_N-1; i>=0; i--){
      if (v >= pgmF(&LUT_VMIN[i])) return (uint8_t)lroundf(pgmF(&LUT_SOC[i]));
    }
    return 1;
  }
}

// ================================================================
// Robustes BMS-Lesen
// ================================================================
bool readBMSFrame(byte cmd[], byte buffer[], size_t buflen, uint8_t &out_len, uint8_t &out_cmd) {
  while (bmsSerial.available()) bmsSerial.read();
  bmsSerial.write(cmd, 7);

  unsigned long t0 = millis();
  int b;

  // Startbyte 0xDD
  do {
    if (millis() - t0 > 300) return false;
    b = bmsSerial.read();
  } while (b != 0xDD);
  buffer[0] = 0xDD;

  // 0xA5 + CMD Echo
  while (bmsSerial.available() < 2) { if (millis()-t0 > 300) return false; }
  buffer[1] = (byte)bmsSerial.read(); // 0xA5
  buffer[2] = (byte)bmsSerial.read(); // 0x03/0x04
  out_cmd = buffer[2];

  // LEN
  while (bmsSerial.available() < 1) { if (millis()-t0 > 300) return false; }
  buffer[3] = (byte)bmsSerial.read();
  out_len = buffer[3];

  // Daten + 2 CRC + 1 Endbyte
  uint16_t need = 4 + out_len + 3;
  if (need > buflen) return false;

  for (uint16_t i = 4; i < need; i++) {
    while (!bmsSerial.available()) { if (millis()-t0 > 300) return false; }
    buffer[i] = (byte)bmsSerial.read();
  }

  if (buffer[need-1] != 0x77) return false;
  return true;
}

// ================================================================
// CAN-Sende-Routine
// ================================================================
inline void serviceCanTick(){
  if (!sendDue) return;
  noInterrupts(); sendDue=false; interrupts();

  // 0x201 – Basisdaten + BMS-SoC
  canMsg.can_id  = 0x201;
  canMsg.can_dlc = 8;
  canMsg.data[0] = packVoltage >> 8;
  canMsg.data[1] = packVoltage & 0xFF;
  canMsg.data[2] = packCurrent >> 8;
  canMsg.data[3] = packCurrent & 0xFF;
  canMsg.data[4] = soc;         // BMS-SoC (roh)
  canMsg.data[5] = bmsTemp;
  canMsg.data[6] = bminus;
  canMsg.data[7] = cellTemp;
  can0.sendMessage(&canMsg);

  // 0x202 – aktuelle Zell-Min/Max + Ströme
  canMsg.can_id  = 0x202;
  canMsg.can_dlc = 8;
  canMsg.data[0] = maxChargeCurrent >> 8;
  canMsg.data[1] = maxChargeCurrent & 0xFF;
  canMsg.data[2] = maxDischargeCurrent >> 8;
  canMsg.data[3] = maxDischargeCurrent & 0xFF;
  canMsg.data[4] = lastMaxCellMv >> 8;
  canMsg.data[5] = lastMaxCellMv & 0xFF;
  canMsg.data[6] = lastMinCellMv >> 8;
  canMsg.data[7] = lastMinCellMv & 0xFF;
  can0.sendMessage(&canMsg);

  // 0x203 – SoH + SoC2 + Debug-Infos
  uint8_t soh_local = (uint8_t)((1.0f - (float(cycleCount) / 5000.0f) / 5.0f) * 100.0f);

  canMsg.can_id  = 0x203;
  canMsg.can_dlc = 8;
  canMsg.data[0] = cycleCount >> 8;
  canMsg.data[1] = cycleCount & 0xFF;
  canMsg.data[2] = soh_local;
  canMsg.data[3] = soc2;      // kalibrierter SoC (mit Offset)

  // Byte 4: LUT-SoC (geglätteter neuer SoC)
  canMsg.data[4] = socLut;

  // Byte 5: aktueller Offset (signed)
  {
    int16_t off = socOffset;
    if (off < -128) off = -128;
    if (off >  127) off =  127;
    canMsg.data[5] = (int8_t)off;
  }

  // Byte 6: letzte Korrektur bei der letzten Ruhekalibrierung (SOC_LUT - SOC_BMS)
  canMsg.data[6] = (int8_t)lastDelta;

  // Byte 7: Statusbits (Bit0 = isResting, Bit1 = restCalibrated)
  uint8_t status = 0;
  if (isResting)      status |= 0x01;
  if (restCalibrated) status |= 0x02;
  canMsg.data[7] = status;

  can0.sendMessage(&canMsg);
}

// ================================================================
void setupTimer1_1Hz(){
  noInterrupts();
  TCCR1A=0; TCCR1B=0; TCNT1=0;
  TCCR1B|=(1<<WGM12);
  OCR1A=15624;                     // 1 Hz @16 MHz/1024
  TCCR1B|=(1<<CS12)|(1<<CS10);
  TIMSK1|=(1<<OCIE1A);
  interrupts();
}
ISR(TIMER1_COMPA_vect){ sendDue=true; }

// ================================================================
void setup(){
  bmsSerial.begin(9600);

  EEPROM.get(EEPROM_MAX_CHARGE, maxChargeCurrent);
  EEPROM.get(EEPROM_MAX_DISCHARGE, maxDischargeCurrent);
  if (maxChargeCurrent    < 0 || maxChargeCurrent    > 20000) maxChargeCurrent    = 0;
  if (maxDischargeCurrent < 0 || maxDischargeCurrent > 20000) maxDischargeCurrent = 0;

  can0.reset();
  can0.setBitrate(CAN_500KBPS, MCP_16MHZ);
  can0.setNormalMode();

  setupTimer1_1Hz();
}

// ================================================================
void loop(){
  static byte data[160];

  serviceCanTick();

  // --- BASIC ---
  {
    uint8_t len, cmd;
    if (readBMSFrame(reqBasic, data, sizeof(data), len, cmd)) {
      packVoltage = (data[4] << 8) | data[5];
      packCurrent = (int16_t)((data[6] << 8) | data[7]);
      soc         = data[23];
      cycleCount  = (data[12] << 8) | data[13];
      bmsTemp     = (((data[27] << 8) | data[28]) - 2731)/10;
      cellTemp    = (((data[29] << 8) | data[30]) - 2731)/10;
      bminus      = (((data[31] << 8) | data[32]) - 2731)/10;

      if (packCurrent > maxChargeCurrent) {
        maxChargeCurrent = packCurrent;
        EEPROM.put(EEPROM_MAX_CHARGE, maxChargeCurrent);
      }
      if (packCurrent < 0 && -packCurrent > maxDischargeCurrent) {
        maxDischargeCurrent = -packCurrent;
        EEPROM.put(EEPROM_MAX_DISCHARGE, maxDischargeCurrent);
      }

      // Offset löschen, wenn BMS voll meldet
      if (soc >= 100) {
        socOffset = 0;
      }
    }
  }

  serviceCanTick();

  // --- CELL ---
  {
    uint8_t len, cmd;
    if (readBMSFrame(reqCell, data, sizeof(data), len, cmd)) {
      int cell_count = (len - 3) / 2;
      if (cell_count < 0)  cell_count = 0;
      if (cell_count > 24) cell_count = 24;

      if (cell_count > 0) {
        uint16_t fmax = 0;
        uint16_t fmin = 0xFFFF;

        for (int i = 0; i < cell_count; i++) {
          int base = 4 + i*2;
          uint16_t cell = (data[base] << 8) | data[base+1]; // mV
          if (cell > fmax) fmax = cell;
          if (cell < fmin) fmin = cell;
        }

        lastMaxCellMv = fmax;
        lastMinCellMv = fmin;

        if (fmax > maxCellVoltage) maxCellVoltage = fmax;
        if (fmin < minCellVoltage) minCellVoltage = fmin;
      }
    }
  }

  serviceCanTick();

  // --- alle ~250 ms: LUT-SoC & Kalibrierlogik ---
  unsigned long now = millis();
  if (now - last250 >= 250){
    last250 = now;

    // Zellmittelwertbildung
    pushCellMv(lastMinCellMv, lastMaxCellMv);

    float minCellMeanV = (cntCell ? (float)sumMin / (float)cntCell : 0.0f) / 1000.0f;
    float maxCellMeanV = (cntCell ? (float)sumMax / (float)cntCell : 0.0f) / 1000.0f;

    // Lade/Entlade-Richtung für LUT
    if (packCurrent >= I_EPS) charging_mode = true;
    else                      charging_mode = false;

    uint8_t socCandidate = charging_mode
      ? lookupSOC_byVoltage(maxCellMeanV, true)
      : lookupSOC_byVoltage(minCellMeanV, false);

    // monotones Halten
    if (socShown < 0) socShown = socCandidate;
    if (charging_mode) {
      if (socCandidate > socShown) socShown = socCandidate;
    } else {
      if (socCandidate < socShown) socShown = socCandidate;
    }

    // 1-Minuten-Mittelwert über LUT-SoC
    pushSoc((uint8_t)socShown);
    uint16_t denom = (cntSoc ? cntSoc : 1);
    uint32_t tmpMean = (sumSoc + (denom / 2)) / denom;
    if (tmpMean > 100) tmpMean = 100;
    socLut = (uint8_t)tmpMean;

    // ---- Ruheerkennung (kein Strom / 5 Minuten) ----
    int16_t absI = (packCurrent >= 0) ? packCurrent : -packCurrent;

    if (absI < I_REST) {
      // Ruhe
      if (!isResting) {
        isResting      = true;
        restCalibrated = false;
        restStartMs    = now;
      }

      // nach 5 Minuten Ruhe: Kalibrierung
      if (!restCalibrated && (now - restStartMs >= REST_TIME_MS)) {
        int16_t delta = (int16_t)socLut - (int16_t)soc;
        socOffset += delta;

        // delta auf int8 begrenzen und merken
        if (delta < -128) delta = -128;
        if (delta >  127) delta =  127;
        lastDelta = (int8_t)delta;

        restCalibrated = true;
      }

    } else {
      // Lastbetrieb
      isResting = false;
      // restCalibrated bleibt, wird bei nächstem Ruhe-Eintritt zurückgesetzt
    }

    // ---- Ausgabe-SoC (soc2) ----
    uint8_t outSoc = 0;
    if (isResting && restCalibrated) {
      // nach 5 Min Ruhe -> direkt LUT-SoC nutzen (ohne Offset)
      outSoc = socLut;
    } else {
      // Last oder Ruhe < 5 Min -> BMS-SoC + Offset
      int16_t s = (int16_t)soc + socOffset;
      if (s < 0)   s = 0;
      if (s > 100) s = 100;
      outSoc = (uint8_t)s;
    }

    soc2 = outSoc;
  }
}
