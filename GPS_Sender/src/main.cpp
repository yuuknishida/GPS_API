// Arduino Nano 

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

#define SCK_PIN     13
#define MISO_PIN    12
#define MOSI_PIN    11
#define CS_PIN      10
#define RESET_PIN   9
#define IRQ_PIN     2

const long frequency = 915E6;

// GPS data storage
struct GPSData {
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

  LoRa.setPins(CS_PIN, RESET_PIN, IRQ_PIN);
  SPI.begin();
  LoRa_SenderMode();

  if (!LoRa.begin(frequency))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
}

void loop()
{
  // Read GPS data character by character
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    
    // Build NMEA sentence (starts with $, ends with newline)
    if (c == '$') {
      nmeaSentence = "$";  // Start new sentence
    }
    else if (c == '\n') {
      // Complete sentence received, parse it
      if (nmeaSentence.length() > 0) {
        parseNMEA(nmeaSentence);
        
        // If we have valid GPS data, send it via LoRa
        if (gpsData.valid) {
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
    else if (c != '\r') {  // Ignore carriage return
      nmeaSentence += c;
    }
  }
}

void LoRa_SenderMode()
{
  LoRa.idle();
  LoRa.disableInvertIQ();
}

void sendMessage(String message)
{
  LoRa_SenderMode();
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket(true);
}

// Parse NMEA sentence
void parseNMEA(String sentence) {
  // Validate checksum first
  if (!validateChecksum(sentence)) {
    return;
  }
  
  // Parse GPGGA sentence (position, altitude, satellites)
  // Format: $GPGGA,time,lat,N/S,lon,E/W,quality,numSat,hdop,alt,M,...*checksum
  if (sentence.startsWith("$GPGGA") || sentence.startsWith("$GNGGA")) {
    int commaPos[14];
    int commaCount = 0;
    
    // Find all comma positions
    for (int i = 0; i < sentence.length() && commaCount < 14; i++) {
      if (sentence.charAt(i) == ',') {
        commaPos[commaCount++] = i;
      }
    }
    
    if (commaCount >= 9) {
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
      if (quality > 0 && latStr.length() > 0 && lonStr.length() > 0) {
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
  else if (sentence.startsWith("$GPRMC") || sentence.startsWith("$GNRMC")) {
    int commaPos[12];
    int commaCount = 0;
    
    for (int i = 0; i < sentence.length() && commaCount < 12; i++) {
      if (sentence.charAt(i) == ',') {
        commaPos[commaCount++] = i;
      }
    }
    
    if (commaCount >= 7) {
      // Extract status (field 2) - must be 'A' for active
      char status = sentence.charAt(commaPos[1] + 1);
      
      // Extract speed in knots (field 7)
      String speedStr = sentence.substring(commaPos[6] + 1, commaPos[7]);
      
      if (status == 'A' && speedStr.length() > 0) {
        // Convert knots to mph (1 knot = 1.15078 mph)
        gpsData.speed = speedStr.toFloat() * 1.15078;
      }
    }
  }
}

// Convert NMEA coordinate format to decimal degrees
// NMEA format: ddmm.mmmm for latitude, dddmm.mmmm for longitude
double nmeaToDecimal(String nmeaCoord, char direction) {
  if (nmeaCoord.length() == 0) return 0.0;
  
  // Find decimal point
  int dotPos = nmeaCoord.indexOf('.');
  if (dotPos < 3) return 0.0;
  
  // Extract degrees (everything before last 2 digits before decimal)
  String degStr = nmeaCoord.substring(0, dotPos - 2);
  double degrees = degStr.toFloat();
  
  // Extract minutes (last 2 digits before decimal + decimal part)
  String minStr = nmeaCoord.substring(dotPos - 2);
  double minutes = minStr.toFloat();
  
  // Convert to decimal degrees
  double decimal = degrees + (minutes / 60.0);
  
  // Apply direction (S and W are negative)
  if (direction == 'S' || direction == 'W') {
    decimal = -decimal;
  }
  
  return decimal;
}

// Validate NMEA checksum
bool validateChecksum(String sentence) {
  // Find * that precedes checksum
  int asteriskPos = sentence.indexOf('*');
  if (asteriskPos < 1) return false;
  
  // Extract checksum (2 hex digits after *)
  if (sentence.length() < asteriskPos + 3) return false;
  String checksumStr = sentence.substring(asteriskPos + 1, asteriskPos + 3);
  int expectedChecksum = strtol(checksumStr.c_str(), NULL, 16);
  
  // Calculate checksum (XOR of all characters between $ and *)
  int calculatedChecksum = 0;
  for (int i = 1; i < asteriskPos; i++) {
    calculatedChecksum ^= sentence.charAt(i);
  }
  
  return (calculatedChecksum == expectedChecksum);
}
