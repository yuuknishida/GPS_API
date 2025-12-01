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

struct GPSData {
  double longitude = 0.0;
  double latitude = 0.0;
  double speed = 0.0;
  float altitude = 0.0;
  int satellites = 0;
  bool isValid = false;
}

GPSData gpsData;

// put function declarations here:
SoftwareSerial gpsSerial(4, 3);
void LoRa_SenderMode();
void sendMessage(String message);
bool parseGPGGA(String sentence);
double convertToDecimalDegrees(String nmeaCoord, char direction);

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
  static String nmeaSentence = "";
 
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    // Serial.write(c);

    if (c=='\n') {
      if (nmeaSentence.startsWith("$GPGGA") || nmeaSentence.startsWith("$GNGGA")) {
        if (parseGPGGA(nmeaSentence) && gpsData.isValid) {
          String gps_str = String(gpsData.latitude, 6) + "," +
                           String(gpsData.longitude, 6) + "," +
                           String(gpsData.altitude, 1) + "," +
                           String(gpsData.satellites);
          sendMessage(gps_str);

          Serial.print("  Latitude: ");
          Serial.println(GPSData.latitude, 6);

          Serial.print("  Longitude: ");
          Serial.println(GPSData.longitude, 6);

          Serial.print("  Altitude: ");
          Serial.println(GPSData.altitude, 6);

          Serial.print("  Satellites: ");
          Serial.println(GPSData.satellites);
        }
      }
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

bool parseGPGGA(String sentence)
{
  int fieldIndex = 0;
  int startPos = 0;
  String fields[15];

  for (int i = 0; i < sentence.length();i++) 
  {
    if (sentence[i] == "," || sentence[i] == '*') {
      fields[fieldIndex++] = sentence.substring(startPos, i);
      startPos = i + 1;
      if (fieldIndex >= 15)
        break;
    }
  }

  if(fields[6].length()==0 || fields[6].toInt() == 0) {
    gpsData.isValid = false;
    return false;
  }

  if(fields[2].length() > 0 && fields[3].length() > 0)
    gpsData.latitude = convertToDecimalDegrees(fields[2], fields[3][0]);

  if(fields[4].length() > 0 && fields[5].length() >0)
    gpsData.longitude = convertToDecimalDegrees(fields[4], fields[5][0]);

  if (fields[7].length() > 0)
    gpsData.satellites = fields[7].toInt();

  if(fields[9].length() > 0)
    gpsData.altitude = fields[9].toFloat();

  gpsData.isValid = true;
  return true;
}

double convertToDecimalDegrees(String nmeaCoord, char direction)
{
  if(nmeaCoord.length() == 0)
    return 0.0;

  int dotPos = nmeaCoord.indexOf('.');
  if(dotPos < 0)
    return 0.0;

  int degreeDigits = (nmeaCoord.length() >= 10) ? 3 : 2;
  String degStr = nmeaCoord.substring(0, degreeDigits);
  String minStr = nmeaCoord.substring(degreeDigits);

  double degrees = degStr.toDouble();
  double minutes = minStr.toDouble();

  double decimalDegrees = degrees + (minutes / 60.0);

  if (direction == 'S' || direction == 'W'){
    decimalDegrees = -decimalDegrees;
  }

  return decimalDegrees;
}
