// Arduino Nano 

#include <Arduino.h>
#include <TinyGPSPlus.h>
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

// put function declarations here:
TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3);
void LoRa_SenderMode();
void sendMessage(String message);

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
  String gps_str = "";
 
  while (gpsSerial.available() > 0)
  {
    char c = gpsSerial.read();
    // Serial.write(c);

    if (gps.encode(c) && gps.location.isValid())
    {
      gps_str = String(gps.location.lat(), 6) + "," +
                String(gps.location.lng(), 6) + "," +
                String(gps.altitude.meters(), 1) + "," +
                String(gps.speed.mph()) + "," +
                String(gps.satellites.value());
      sendMessage(gps_str);
      
      Serial.print("   Latitude: ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("   Longitude: ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("   Altitude: ");
      Serial.print(gps.altitude.meters());
      Serial.println("m");

      Serial.print("   Speed: ");
      Serial.print(gps.speed.mph());
      Serial.println(" mph");

      Serial.print("   Satellites: ");
      Serial.println(gps.satellites.value());
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
