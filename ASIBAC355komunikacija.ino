#include <Arduino.h>

HardwareSerial bacSerial(2);

// UART
static const uint32_t BAC_BAUD = 115200; // po potrebi spremeni
static const int BAC_RX_PIN = 16;        // ESP32 RX2 <- BAC TX
static const int BAC_TX_PIN = 17;        // ESP32 TX2 -> BAC RX

// Modbus
static const uint8_t SLAVE_ID = 0x01;     // po potrebi spremeni
static const uint16_t VBAT_ADDR = 265;    // samo ta register (napetost), za temperaturo pa damo 261 
static const uint16_t REG_COUNT = 1;
static const float SCALE = 32.0f;         // podan scale za napetost, 

static const uint32_t POLL_MS = 1000;
uint32_t lastPoll = 0;

// CRC16 Modbus
uint16_t modbusCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

void sendRead265() {
  uint8_t req[8];
  req[0] = SLAVE_ID;
  req[1] = 0x03; // Read Holding Registers
  req[2] = (uint8_t)(VBAT_ADDR >> 8);
  req[3] = (uint8_t)(VBAT_ADDR & 0xFF);
  req[4] = 0x00;
  req[5] = 0x01; // 1 register

  uint16_t crc = modbusCRC(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);        // CRC low
  req[7] = (uint8_t)((crc >> 8) & 0xFF); // CRC high

  bacSerial.write(req, sizeof(req));
  bacSerial.flush();
}

size_t readResponse(uint8_t* buf, size_t maxLen, uint32_t timeoutMs = 120) {
  size_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (bacSerial.available() && n < maxLen) {
      buf[n++] = (uint8_t)bacSerial.read();
      t0 = millis();
    }
    delay(1);
  }
  return n;
}

bool parseVbat(const uint8_t* rx, size_t len, float& vbat) {
  // Pričakovan odgovor: [ID][03][02][DATA_H][DATA_L][CRC_L][CRC_H]
  if (len < 7) return false;
  if (rx[0] != SLAVE_ID) return false;

  // Exception odgovor
  if (rx[1] == 0x83) return false;
  if (rx[1] != 0x03) return false;
  if (rx[2] != 0x02) return false;

  uint16_t crcCalc = modbusCRC(rx, len - 2);
  uint16_t crcRecv = (uint16_t)rx[len - 2] | ((uint16_t)rx[len - 1] << 8);
  if (crcCalc != crcRecv) return false;

  uint16_t raw = ((uint16_t)rx[3] << 8) | rx[4];
  vbat = raw / SCALE;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  bacSerial.begin(BAC_BAUD, SERIAL_8N1, BAC_RX_PIN, BAC_TX_PIN);

  Serial.println("BAC355 VBAT reader: addr 265, scale 32");
}

void loop() {
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();

    sendRead265();

    uint8_t rx[64];
    size_t n = readResponse(rx, sizeof(rx));

    if (n == 0) {
      Serial.println("Ni odgovora.");
      return;
    }

    float vbat = 0.0f;
    if (parseVbat(rx, n, vbat)) {
      Serial.print("VBAT: ");
      Serial.print(vbat, 2);
      Serial.println(" V");
    } else {
      Serial.println("Napaka pri branju/parsanju.");
    }
  }
}
