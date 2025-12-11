// Arduino Nano ESP32
#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>

// registers
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

// PIN SPI DEFINITIONS
#define SCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10
#define RESET_PIN 9
#define IRQ_PIN 2

const long frequency = 915E6;
String data_str;

// Toggle this to switch between local and cloud
#define USE_CLOUD true // Set to false for local testing

#if USE_CLOUD
const char *serverURL = "https://gps-tracker-backend-fqq3.onrender.com/gps"; // UPDATE THIS!
#else
const char *serverURL = "http://192.168.1.17:5000/gps";
#endif

// put function declarations here:
String ssid = "SpectrumSetup-CF";
String password = "tabletdomain104";

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Receiver setup starting...");

  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20)
  {
    Serial.print(".");
    delay(500);
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected!");
    Serial.print("Local IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nFailed to connect.");
  }

  Serial.println("Initializing LoRa");
  if (!rfm95_init(frequency))
  {
    Serial.println("Starting LoRa failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("LoRa initialized successfully");
  }
  rfm95_writeReg(REG_FIFO_RX_BASE_ADDR, 0x80);
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0x80);
  rfm95_writeReg(0x1E, 0x74 | 0x04); // SF7 + CRC
  rfm95_setOpsMode(MODE_RX_CONTINUOUS);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("Checking for packets...");
  int parseSize = rfm95_checkpacket();
  if (parseSize > 0)
  {
    char msg[256];
    rfm95_readPacket(msg, parseSize);
    msg[parseSize] = '\0';
    String received = msg;
    Serial.print("Received: ");
    Serial.println(msg);

    if (received.startsWith("<") && received.endsWith(">"))
    {
      received = received.substring(1, received.length() - 1);

      // latitude,longitude,altitude,speed,satellites
      int comma1 = received.indexOf(',');
      int comma2 = received.indexOf(',', comma1 + 1);
      int comma3 = received.indexOf(',', comma2 + 1);
      int comma4 = received.indexOf(',', comma3 + 1);

      if (comma1 != -1 &&
          comma2 != -1 &&
          comma3 != -1 &&
          comma4 != -1)
      {
        double latitude = received.substring(0, comma1).toDouble();
        double longitude = received.substring(comma1 + 1, comma2).toDouble();
        double altitude = received.substring(comma2 + 1, comma3).toDouble();
        double speed = received.substring(comma3 + 1, comma4).toDouble();
        double satellites = received.substring(comma4 + 1).toDouble();

        Serial.printf("Lat: %.6f  Lon: %.6f  Alt: %.1f m  Speed: %.2f mph  Sat: %.0f\n",
                      latitude, longitude, altitude, speed, satellites);

        if (WiFi.status() == WL_CONNECTED)
        {
          HTTPClient http;
          String jsonPayLoad = "{\"latitude\":" + String(latitude, 6) +
                               ",\"longitude\":" + String(longitude, 6) +
                               ",\"altitude\":" + String(altitude, 1) +
                               ",\"speed\":" + String(speed) +
                               ",\"satellites\":" + String(satellites) + "}";

          http.begin(serverURL);
          http.addHeader("Content-Type", "application/json");
          int httpResponseCode = http.POST(jsonPayLoad);
          Serial.println("HTTP Response Code: " + String(httpResponseCode));
          http.end();
        }
      }
    }
  }
}

// ---------------- LoRa helper functions ----------------
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
  rfm95_setOpsMode(MODE_STDBY);
  return true;
}

int rfm95_checkpacket()
{
  uint8_t irqFlags = rfm95_readReg(REG_IRQ_FLAGS);
  if (irqFlags & IRQ_RX_DONE_MASK)
  {
    int len = rfm95_readReg(REG_RX_NB_BYTES);
    rfm95_writeReg(REG_IRQ_FLAGS, IRQ_RX_DONE_MASK); // clear flag
    rfm95_writeReg(REG_FIFO_ADDR_PTR, rfm95_readReg(REG_FIFO_RX_CURRENT_ADDR));
    return len;
  }
  return 0;
}

void rfm95_readPacket(char *buffer, int packetLength)
{
  rfm95_writeReg(REG_FIFO_ADDR_PTR, rfm95_readReg(REG_FIFO_RX_CURRENT_ADDR));
  for (int i = 0; i < packetLength; i++)
  {
    buffer[i] = (char)rfm95_readReg(REG_FIFO);
  }
  buffer[packetLength] = '\0';
}