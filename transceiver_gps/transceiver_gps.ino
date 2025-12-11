// Arduino Nano

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_TX_NB_BYTES 0x14
#define REG_PAYLOAD_LENGTH 0x22
#define REG_VERSION 0x42

#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05

#define IRQ_TX_DONE_MASK 0x08

#define SCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10
#define RESET_PIN 9
#define IRQ_PIN 2

const long frequency = 915E6;

// GPS data storage
struct GPSData
{
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double speed = 0.0;
  int satellites = 0;
  bool valid = false;
};

GPSData gpsData;
TinyGPSPlus gps;

char gps_buf[64];

void LoRa_SenderMode();
void sendMessage(String message);
void parseNMEA(String sentence);
double nmeaToDecimal(String nmeaCoord, char direction);
bool validateChecksum(String sentence);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println("Initializing LoRa...");
  if (!rfm95_init(frequency))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ;
  }

  rfm95_writeReg(REG_FIFO_TX_BASE_ADDR, 0x00);
  rfm95_writeReg(REG_FIFO_RX_BASE_ADDR, 0x80);
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0x00);

  rfm95_writeReg(0x1e, 0xA4 | 0x04);

  rfm95_setOpsMode(MODE_STDBY);
}

void loop()
{
  // Read GPS data character by character
  while (Serial1.available())
  {
    char c = Serial1.read();
    // Serial.print(c);

    gps.encode(c);
  }

  if (gps.location.isValid())
  {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    double spd = gps.speed.mph();
    double alt = gps.altitude.meters();
    int satellites = gps.satellites.value();

    snprintf(gps_buf, sizeof(gps_buf), "<%.6f,%.6f,%.2f,%.2f,%d>", lat, lng, alt, spd, satellites);
    // String gps_str = "<" + String(lat, 6) + "," +
    //                        String(lng, 6) + "," +
    //                        String(alt, 1) + "," +
    //                        String(spd, 2) + "," +
    //                        String(satellites) + ">";

    Serial.println("\nSending: " + String(gps_buf));
    rfm95_sendPacket(gps_buf);
    delay(1000);
  }
  else
  {
    Serial.println("\nWaiting for GPS fix...");
    Serial.print("Satellites in view: ");
    Serial.println(gps.satellites.value());
  }
}

void LoRa_SenderMode()
{
  // Disable InvertIQ for normal transmission
  rfm95_writeReg(0x33, 0x27);
  rfm95_writeReg(0x3b, 0x1D);

  rfm95_writeReg(REG_FIFO_TX_BASE_ADDR, 0x00);
  rfm95_writeReg(REG_FIFO_RX_BASE_ADDR, 0x80);
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0x00);

  // Set to standby mode (ready to transmit)
  rfm95_setOpsMode(MODE_STDBY);
}

void sendMessage(String message)
{
  // Ensure we're in sender mode
  rfm95_setOpsMode(MODE_STDBY);

  // Get message length
  int messageLength = message.length();
  if (messageLength > 255)
    messageLength = 255; // Max payload

  // Set FIFO address pointer to TX base
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0x00);
  // Write message to FIFO
  for (int i = 0; i < messageLength; i++)
  {
    rfm95_writeReg(REG_FIFO, message.charAt(i));
  }

  // Set payload length
  rfm95_writeReg(REG_PAYLOAD_LENGTH, messageLength);

  // Switch to TX mode to transmit
  rfm95_setOpsMode(MODE_TX);

  // Wait for transmission to complete (TX_DONE flag)
  while (!(rfm95_readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK))
    ;

  // Clear IRQ flags
  rfm95_writeReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  // Return to standby mode
  rfm95_setOpsMode(MODE_STDBY);
}

// Function declarations
void rfm95_writeReg(uint8_t address, uint8_t value)
{
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

uint8_t rfm95_readReg(uint8_t address)
{
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address & 0x7f);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void rfm95_hardwareReset()
{
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
}

void rfm95_setFrequency(long frequency)
{
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  rfm95_writeReg(REG_FRF_MSB, frf >> 16);
  rfm95_writeReg(REG_FRF_MID, frf >> 8);
  rfm95_writeReg(REG_FRF_LSB, frf >> 0);
}

void rfm95_setOpsMode(uint8_t mode)
{
  rfm95_writeReg(REG_OP_MODE, MODE_LONG_RANGE | mode);
}

bool rfm95_init(long frequency)
{
  pinMode(CS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  rfm95_hardwareReset();

  uint8_t version = rfm95_readReg(REG_VERSION);
  if (version != 0x12)
    return false;

  rfm95_setOpsMode(MODE_SLEEP);
  rfm95_setFrequency(frequency);

  rfm95_writeReg(REG_PA_CONFIG, 0xFF);
  rfm95_writeReg(REG_LNA, 0x23);
  rfm95_setOpsMode(MODE_STDBY);

  return true;
}

void rfm95_sendPacket(const char *data)
{
  int len = strlen(data);
  if (len > 255)
    len = 255;

  rfm95_setOpsMode(MODE_STDBY);
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0x00);

  for (int i = 0; i < len; i++)
  {
    rfm95_writeReg(REG_FIFO, data[i]);
  }

  rfm95_writeReg(REG_PAYLOAD_LENGTH, len);
  rfm95_setOpsMode(MODE_TX);

  while (!(rfm95_readReg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK))
  {
    yield();
  }

  rfm95_writeReg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  rfm95_setOpsMode(MODE_STDBY);
}