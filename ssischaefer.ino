#include <SPI.h>
#include <mcp2515.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// --------- CAN & BMS ----------
MCP2515 can0(10);
SoftwareSerial bmsSerial(3, 4);

// --------- EEPROM ----------
#define EEPROM_MAX_CHARGE    0
#define EEPROM_MAX_DISCHARGE 2

// --------- BMS Requests ----------
byte reqBasic[] = {0xDD,0xA5,0x03,0x00,0xFF,0xFD,0x77};
byte reqCell[]  = {0xDD,0xA5,0x04,0x00,0xFF,0xFC,0x77};

// --------- Variablen ----------
uint16_t packVoltage = 0;
int16_t  packCurrent = 0;
uint8_t  soc = 0;
int8_t   bmsTemp = 0;
int8_t   boardTemp = 0;
int8_t   cellTemp = 0;

uint16_t maxCellVoltage = 0;
uint16_t minCellVoltage = 0xFFFF;

uint16_t lastMaxCell = 0;
uint16_t lastMinCell = 0;

int16_t maxChargeCurrent = 0;
int16_t maxDischargeCurrent = 0;

uint16_t cycleCount = 0;
uint8_t soh = 0;

volatile bool sendDue = false;
struct can_frame canMsg;

// ================================================================
// Robustes BMS-Lesen (vom neuen Code)
// ================================================================
bool readBMSFrame(byte cmd[], byte buffer[], size_t buflen, uint8_t &out_len, uint8_t &out_cmd)
{
  while (bmsSerial.available()) bmsSerial.read();
  bmsSerial.write(cmd, 7);

  unsigned long t0 = millis();
  int b;

  // Startbyte
  do {
    if (millis() - t0 > 300) return false;
    b = bmsSerial.read();
  } while (b != 0xDD);
  buffer[0] = 0xDD;

  // Header
  while (bmsSerial.available() < 2) { if (millis()-t0 > 300) return false; }
  buffer[1] = bmsSerial.read();
  buffer[2] = bmsSerial.read();
  out_cmd = buffer[2];

  // Länge
  while (bmsSerial.available() < 1) { if (millis()-t0 > 300) return false; }
  buffer[3] = bmsSerial.read();
  out_len = buffer[3];

  uint16_t need = 4 + out_len + 3;
  if (need > buflen) return false;

  for (uint16_t i = 4; i < need; i++) {
    while (!bmsSerial.available()) { if (millis()-t0 > 300) return false; }
    buffer[i] = bmsSerial.read();
  }

  return (buffer[need-1] == 0x77);
}

// ================================================================
// CAN via Timer 1 Hz
// ================================================================
void setupTimer1_1Hz() {
  noInterrupts();
  TCCR1A = 0; TCCR1B = 0; TCNT1 = 0;
  TCCR1B |= (1 << WGM12);
  OCR1A = 15624;
  TCCR1B |= (1 << CS12) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}
ISR(TIMER1_COMPA_vect) { sendDue = true; }

// ================================================================
void setup() {
  bmsSerial.begin(9600);

  EEPROM.get(EEPROM_MAX_CHARGE, maxChargeCurrent);
  EEPROM.get(EEPROM_MAX_DISCHARGE, maxDischargeCurrent);
  if (maxChargeCurrent < 0 || maxChargeCurrent > 20000) maxChargeCurrent = 0;
  if (maxDischargeCurrent < 0 || maxDischargeCurrent > 20000) maxDischargeCurrent = 0;

  can0.reset();
  can0.setBitrate(CAN_500KBPS, MCP_16MHZ);
  can0.setNormalMode();

  setupTimer1_1Hz();
}

// ================================================================
void loop() {
  static byte data[160];
  uint8_t len, cmd;

  // --- BASIC ---
  if (readBMSFrame(reqBasic, data, sizeof(data), len, cmd)) {
    packVoltage = (data[4] << 8) | data[5];
    packCurrent = (int16_t)((data[6] << 8) | data[7]);
    soc         = data[23];
    cycleCount  = (data[12] << 8) | data[13];

    bmsTemp   = (((data[27] << 8) | data[28]) / 10) - 40;
    boardTemp = (((data[29] << 8) | data[30]) / 10) - 40;
    cellTemp  = (((data[31] << 8) | data[32]) / 10) - 40;

    if (packCurrent > maxChargeCurrent) {
      maxChargeCurrent = packCurrent;
      EEPROM.put(EEPROM_MAX_CHARGE, maxChargeCurrent);
    }
    if (packCurrent < 0 && -packCurrent > maxDischargeCurrent) {
      maxDischargeCurrent = -packCurrent;
      EEPROM.put(EEPROM_MAX_DISCHARGE, maxDischargeCurrent);
    }
  }

  // --- CELL ---
  if (readBMSFrame(reqCell, data, sizeof(data), len, cmd)) {
    uint16_t fmax = 0;
    uint16_t fmin = 0xFFFF;

    int cell_count = (len - 3) / 2;
    if (cell_count > 24) cell_count = 24;

    for (int i = 0; i < cell_count; i++) {
      uint16_t v = (data[4 + i*2] << 8) | data[5 + i*2];
      if (v > fmax) fmax = v;
      if (v < fmin) fmin = v;
    }

    lastMaxCell = fmax;
    lastMinCell = fmin;

    if (fmax > maxCellVoltage) maxCellVoltage = fmax;
    if (fmin < minCellVoltage) minCellVoltage = fmin;
  }

  // ============================================================
  // CAN SEND 1x pro Sekunde via Timer
  // ============================================================
  if (!sendDue) return;
  noInterrupts(); sendDue = false; interrupts();

  // 0x201
  canMsg.can_id = 0x201;
  canMsg.can_dlc = 8;
  canMsg.data[0] = packVoltage >> 8;
  canMsg.data[1] = packVoltage & 0xFF;
  canMsg.data[2] = packCurrent >> 8;
  canMsg.data[3] = packCurrent & 0xFF;
  canMsg.data[4] = soc;
  canMsg.data[5] = bmsTemp;
  canMsg.data[6] = boardTemp;
  canMsg.data[7] = cellTemp;
  can0.sendMessage(&canMsg);

  // 0x202
  canMsg.can_id = 0x202;
  canMsg.data[0] = maxChargeCurrent >> 8;
  canMsg.data[1] = maxChargeCurrent & 0xFF;
  canMsg.data[2] = maxDischargeCurrent >> 8;
  canMsg.data[3] = maxDischargeCurrent & 0xFF;
  canMsg.data[4] = lastMaxCell >> 8;
  canMsg.data[5] = lastMaxCell & 0xFF;
  canMsg.data[6] = lastMinCell >> 8;
  canMsg.data[7] = lastMinCell & 0xFF;
  can0.sendMessage(&canMsg);

  // 0x203
  soh = (1 - ((float)cycleCount / 5000.0) / 5.0) * 100;

  canMsg.can_id = 0x203;
  canMsg.data[0] = cycleCount >> 8;
  canMsg.data[1] = cycleCount & 0xFF;
  canMsg.data[2] = soh;
  canMsg.data[3] = 0;
  canMsg.data[4] = 0;
  canMsg.data[5] = 0;
  canMsg.data[6] = 0;
  canMsg.data[7] = 0;
  can0.sendMessage(&canMsg);
}
