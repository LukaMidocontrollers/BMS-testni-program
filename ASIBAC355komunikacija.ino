#include <Arduino.h>
#include <JKBMSInterface.h>

HardwareSerial bacSerial(1);

// Create BMS instance using Serial2
JKBMSInterface bms(&Serial2);

// UART CONFIG
static const uint32_t BAC_BAUD = 115200;   
static const int BAC_RX_PIN = 47;
static const int BAC_TX_PIN = 48;

// MODBUS CONFIG
static const uint8_t SLAVE_ID = 0x01;
static const uint16_t VBAT_ADDR = 260;
static const float SCALE = 256.0f;

static const uint32_t POLL_MS = 1000;
uint32_t lastPoll = 0;

// ---------------- CRC ----------------
uint16_t modbusCRC(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

// ---------------- REQUEST ----------------
void sendReadVBAT() {
  uint8_t req[8];

  req[0] = SLAVE_ID;
  req[1] = 0x03;
  req[2] = (VBAT_ADDR >> 8) & 0xFF;
  req[3] = VBAT_ADDR & 0xFF;
  req[4] = 0x00;
  req[5] = 0x01;

  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  bacSerial.write(req, 8);
}

// ---------------- READ RESPONSE ----------------
size_t readResponse(uint8_t* buf, size_t maxLen, uint32_t timeout = 300) {
  size_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < timeout) {
    while (bacSerial.available() && idx < maxLen) {
      buf[idx++] = bacSerial.read();
      start = millis(); // extend timeout while receiving
    }
  }

  return idx;
}

// ---------------- PARSE ----------------
bool parseVBAT(const uint8_t* rx, size_t len, float &vbat) {
  if (len < 7) return false;
  if (rx[0] != SLAVE_ID) return false;
  if (rx[1] == 0x83) return false; // exception
  if (rx[1] != 0x03) return false;

  uint16_t crcCalc = modbusCRC(rx, len - 2);
  uint16_t crcRecv = rx[len - 2] | (rx[len - 1] << 8);
  if (crcCalc != crcRecv) return false;

  uint16_t raw = (rx[3] << 8) | rx[4];
  vbat = raw / SCALE;

  return true;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(2000);

  bacSerial.begin(BAC_BAUD, SERIAL_8N1, BAC_RX_PIN, BAC_TX_PIN);
  Serial2.begin(115200, SERIAL_8N1, 20, 21); //bms 

  Serial.println("BAC355 Modbus start");
  Serial.println("JK-BMS Modbus start");
}

// ---------------- LOOP ----------------
void loop() {
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();

    // clear old data
    while (bacSerial.available()) bacSerial.read();

    delay(5); // Modbus silence gap

    sendReadVBAT();

    uint8_t rx[64];
    size_t n = readResponse(rx, sizeof(rx), 300);

    if (n == 0) {
      Serial.println("No response");
      return;
    }

    /*Serial.print("RX bytes: ");
    for (size_t i = 0; i < n; i++) {
      Serial.print(rx[i], HEX);
      Serial.print(" ");
    }
    Serial.println();*/

    float mtemp;
    if (parseVBAT(rx, n, mtemp)) {
      Serial.print("Motor hitrost: ");
      Serial.print(mtemp, 2);
      Serial.println(" km/h");
    } else {
      Serial.println("Parse error");
    }
        
        
    bms.update();
    
    // Check if we have valid data
    if (bms.isDataValid()) {
        // Access battery data
        float voltage = bms.getVoltage();
        uint8_t soc = bms.getSOC();
        float current = bms.getCurrent();
        
        Serial.print("Napetost: ");
        Serial.print(voltage, 2);
        Serial.print("V, SOC: ");
        Serial.print(soc);
        Serial.print("%, Tok: ");
        Serial.print(current, 2);
        Serial.println("A");

        static unsigned long lastSummary = 0;
        if (millis() - lastSummary > 10000) {
            bms.printSummary();
            lastSummary = millis();
        }
        

    } else {
        Serial.println("Čakanje na bms podatki...");
    }

  }
}
