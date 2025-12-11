// Arduino Nano

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>

// registers
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0cu
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_RSSI_VALUE 0x1b
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_FREQ_ERROR_MSB 0x28
#define REG_FREQ_ERROR_MID 0x29
#define REG_FREQ_ERROR_LSB 0x2a
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_INVERTIQ 0x33
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_INVERTIQ2 0x3b
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42
#define REG_PA_DAC 0x4d

// modes
#define MODE_LONG_RANGE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

// PA config
#define PA_BOOST 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

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
String nmeaSentence = "";

// Function declarations
void rfm95_writeReg(uint8_t address, uint8_t value)
{
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t rfm95_readReg(uint8_t address)
{
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address & 0x7f);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
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
  rfm95_writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
  rfm95_writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
  rfm95_writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
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

  rfm95_hardwareReset();

  uint8_t version = rfm95_readReg(REG_VERSION);
  if (version != 0x12)
  {
    return false;
  }

  rfm95_setOpsMode(MODE_SLEEP);
  rfm95_setFrequency(frequency);

  rfm95_writeReg(REG_FIFO_TX_BASE_ADDR, 0);
  rfm95_writeReg(REG_FIFO_RX_BASE_ADDR, 0);
  rfm95_writeReg(REG_LNA, rfm95_readReg(REG_LNA) | 0x03);
  rfm95_writeReg(REG_MODEM_CONFIG_3, 0x04);
  rfm95_writeReg(REG_PA_CONFIG, 0x80 | 0x0f);
  rfm95_writeReg(REG_SYNC_WORD, 0x12);
  rfm95_writeReg(REG_MODEM_CONFIG_1, 0x72);
  rfm95_writeReg(REG_MODEM_CONFIG_2, 0x74);
  rfm95_writeReg(REG_PREAMBLE_MSB, 0x00);
  rfm95_writeReg(REG_PREAMBLE_LSB, 0x08);

  rfm95_setOpsMode(MODE_STDBY);

  return true;
}

int rfm95_checkpacket()
{
  uint8_t irqFlags = rfm95_readReg(REG_IRQ_FLAGS);
  rfm95_writeReg(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && !(irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK))
  {
    int packetLength = rfm95_readReg(REG_RX_NB_BYTES);
    rfm95_writeReg(REG_FIFO_ADDR_PTR, rfm95_readReg(REG_FIFO_RX_CURRENT_ADDR));
    rfm95_setOpsMode(MODE_RX_CONTINUOUS);
    return packetLength;
  }
  else if (rfm95_readReg(REG_OP_MODE) != (MODE_LONG_RANGE | MODE_RX_CONTINUOUS))
  {
    rfm95_writeReg(REG_FIFO_ADDR_PTR, 0);
    rfm95_setOpsMode(MODE_RX_CONTINUOUS);
  }
  return 0;
}

int rfm95_readByte()
{
  return rfm95_readReg(REG_FIFO);
}

int rfm95_bytesAvailable()
{
  return (rfm95_readReg(REG_RX_NB_BYTES) - rfm95_readReg(REG_FIFO_ADDR_PTR));
}

SoftwareSerial gpsSerial(4, 3);
void LoRa_SenderMode();
void sendMessage(String message);
void parseNMEA(String sentence);
double nmeaToDecimal(String nmeaCoord, char direction);
bool validateChecksum(String sentence);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  gpsSerial.begin(9600);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16); // 8 MHz for 16 MHz Arduino

  Serial.println("Initializing LoRa...");

  if (!rfm95_init(frequency))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
      ;
  }
}

void loop()
{
  // Read GPS data character by character
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();

    // Build NMEA sentence (starts with $, ends with newline)
    if (c == '$')
    {
      nmeaSentence = "$"; // Start new sentence
    }
    else if (c == '\n')
    {
      // Complete sentence received, parse it
      if (nmeaSentence.length() > 0)
      {
        parseNMEA(nmeaSentence);

        // If we have valid GPS data, send it via LoRa
        if (gpsData.valid)
        {
          String gps_str = String(gpsData.latitude, 6) + "," +
                           String(gpsData.longitude, 6) + "," +
                           String(gpsData.altitude, 1) + "," +
                           String(gpsData.speed, 2) + "," +
                           String(gpsData.satellites);
          sendMessage(gps_str);

          Serial.print("   Latitude: ");
          Serial.println(gpsData.latitude, 6);
          Serial.print("   Longitude: ");
          Serial.println(gpsData.longitude, 6);
          Serial.print("   Altitude: ");
          Serial.print(gpsData.altitude);
          Serial.println("m");
          Serial.print("   Speed: ");
          Serial.print(gpsData.speed);
          Serial.println(" mph");
          Serial.print("   Satellites: ");
          Serial.println(gpsData.satellites);
        }
      }
      nmeaSentence = "";
    }
    else if (c != '\r')
    { // Ignore carriage return
      nmeaSentence += c;
    }
  }
}

void LoRa_SenderMode()
{
  // Disable InvertIQ for normal transmission
  rfm95_writeReg(REG_INVERTIQ, 0x27);
  rfm95_writeReg(REG_INVERTIQ2, 0x1D);

  // Set to standby mode (ready to transmit)
  rfm95_setOpsMode(MODE_STDBY);
}

void sendMessage(String message)
{
  // Ensure we're in sender mode
  LoRa_SenderMode();

  // Get message length
  int messageLength = message.length();
  if (messageLength > 255)
    messageLength = 255; // Max payload

  // Set FIFO address pointer to TX base
  rfm95_writeReg(REG_FIFO_ADDR_PTR, 0);

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

// Parse NMEA sentence
void parseNMEA(String sentence)
{
  // Validate checksum first
  if (!validateChecksum(sentence))
  {
    return;
  }

  // Parse GPGGA sentence (position, altitude, satellites)
  // Format: $GPGGA,time,lat,N/S,lon,E/W,quality,numSat,hdop,alt,M,...*checksum
  if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA"))
  {
    int commaPos[14];
    int commaCount = 0;

    // Find all comma positions
    for (int i = 0; i < sentence.length() && commaCount < 14; i++)
    {
      if (sentence.charAt(i) == ',')
      {
        commaPos[commaCount++] = i;
      }
    }

    if (commaCount >= 9)
    {
      // Extract latitude (field 2)
      String latStr = sentence.substring(commaPos[1] + 1, commaPos[2]);
      char latDir = sentence.charAt(commaPos[2] + 1);

      // Extract longitude (field 4)
      String lonStr = sentence.substring(commaPos[3] + 1, commaPos[4]);
      char lonDir = sentence.charAt(commaPos[4] + 1);

      // Extract quality (field 6)
      String qualityStr = sentence.substring(commaPos[5] + 1, commaPos[6]);
      int quality = qualityStr.toInt();

      // Extract number of satellites (field 7)
      String satStr = sentence.substring(commaPos[6] + 1, commaPos[7]);

      // Extract altitude (field 9)
      String altStr = sentence.substring(commaPos[8] + 1, commaPos[9]);

      // Only update if we have a valid fix (quality > 0)
      if (quality > 0 && latStr.length() > 0 && lonStr.length() > 0)
      {
        gpsData.latitude = nmeaToDecimal(latStr, latDir);
        gpsData.longitude = nmeaToDecimal(lonStr, lonDir);
        gpsData.altitude = altStr.toFloat();
        gpsData.satellites = satStr.toInt();
        gpsData.valid = true;
      }
    }
  }

  // Parse GPRMC sentence (speed)
  // Format: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,...*checksum
  else if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC"))
  {
    int commaPos[12];
    int commaCount = 0;

    for (int i = 0; i < sentence.length() && commaCount < 12; i++)
    {
      if (sentence.charAt(i) == ',')
      {
        commaPos[commaCount++] = i;
      }
    }

    if (commaCount >= 7)
    {
      // Extract status (field 2) - must be 'A' for active
      char status = sentence.charAt(commaPos[1] + 1);

      // Extract speed in knots (field 7)
      String speedStr = sentence.substring(commaPos[6] + 1, commaPos[7]);

      if (status == 'A' && speedStr.length() > 0)
      {
        // Convert knots to mph (1 knot = 1.15078 mph)
        gpsData.speed = speedStr.toFloat() * 1.15078;
      }
    }
  }
}

// Convert NMEA coordinate format to decimal degrees
// NMEA format: ddmm.mmmm for latitude, dddmm.mmmm for longitude
double nmeaToDecimal(String nmeaCoord, char direction)
{
  if (nmeaCoord.length() == 0)
    return 0.0;

  // Find decimal point
  int dotPos = nmeaCoord.indexOf('.');
  if (dotPos < 3)
    return 0.0;

  // Extract degrees (everything before last 2 digits before decimal)
  String degStr = nmeaCoord.substring(0, dotPos - 2);
  double degrees = degStr.toFloat();

  // Extract minutes (last 2 digits before decimal + decimal part)
  String minStr = nmeaCoord.substring(dotPos - 2);
  double minutes = minStr.toFloat();

  // Convert to decimal degrees
  double decimal = degrees + (minutes / 60.0);

  // Apply direction (S and W are negative)
  if (direction == 'S' || direction == 'W')
  {
    decimal = -decimal;
  }

  return decimal;
}

// Validate NMEA checksum
bool validateChecksum(String sentence)
{
  // Find * that precedes checksum
  int asteriskPos = sentence.indexOf('*');
  if (asteriskPos < 1)
    return false;

  // Extract checksum (2 hex digits after *)
  if (sentence.length() < asteriskPos + 3)
    return false;
  String checksumStr = sentence.substring(asteriskPos + 1, asteriskPos + 3);
  int expectedChecksum = strtol(checksumStr.c_str(), NULL, 16);

  // Calculate checksum (XOR of all characters between $ and *)
  int calculatedChecksum = 0;
  for (int i = 1; i < asteriskPos; i++)
  {
    calculatedChecksum ^= sentence.charAt(i);
  }

  return (calculatedChecksum == expectedChecksum);
}
