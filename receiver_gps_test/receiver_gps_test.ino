// Receiver GPS Module
#include <Arduino.h>
#include <SPI.h>

// LoRa registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_VERSION 0x42

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_RX_CONTINUOUS 0x05

#define IRQ_RX_DONE_MASK 0x40

#define SCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10
#define RESET_PIN 9

const long frequency = 915E6;

// ---------------- LoRa helper functions ----------------
void rfm95_writeReg(uint8_t address, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t rfm95_readReg(uint8_t address) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address & 0x7f);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return value;
}

void rfm95_hardwareReset() {
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
}

void rfm95_setFrequency(long frequency) {
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  rfm95_writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  rfm95_writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
  rfm95_writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void rfm95_setOpsMode(uint8_t mode) {
  rfm95_writeReg(REG_OP_MODE, MODE_LONG_RANGE | mode);
}

bool rfm95_init(long frequency) {
  pinMode(CS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4E6, MSBFIRST, SPI_MODE0));
  rfm95_hardwareReset();

  uint8_t version = rfm95_readReg(REG_VERSION);
  if (version != 0x12) return false;

  rfm95_setOpsMode(MODE_SLEEP);
  rfm95_setFrequency(frequency);

  rfm95_writeReg(REG_FIFO_RX_BASE_ADDR, 0x00);
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0x00);

  rfm95_setOpsMode(MODE_RX_CONTINUOUS);
  return true;
}

int rfm95_checkpacket() {
  uint8_t irqFlags = rfm95_readReg(REG_IRQ_FLAGS);
  if (irqFlags & IRQ_RX_DONE_MASK) {
    int len = rfm95_readReg(REG_RX_NB_BYTES);
    rfm95_writeReg(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK); // clear flag
    return len;
  }
  return 0;
}

void rfm95_readPacket(char* buffer, int packetLength) {
  rfm95_writeReg(REG_FIFO_ADDR_PTR, rfm95_readReg(REG_FIFO_RX_CURRENT_ADDR));
  for(int i = 0; i < packetLength; i++) {
    buffer[i] = (char)rfm95_readReg(REG_FIFO);
  }
  buffer[packetLength] = '\0';
}

// ---------------- Setup & Loop ----------------
void setup() {
  Serial.begin(115200);
  if (!rfm95_init(frequency)) {
    Serial.println("LoRa init failed");
    while(1);
  }
  Serial.println("LoRa Receiver ready");
}

void loop() {
  int packetLength = rfm95_checkpacket();
  if (packetLength > 0) {
    char msg[256];
    rfm95_readPacket(msg, packetLength);
    Serial.print("Received: ");
    Serial.println(msg);
  }
  delay(500);
}
