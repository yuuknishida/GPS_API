// Arduino Nano ESP32

#include <Arduino.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WiFiCredentials.h>
#include <HTTPClient.h>

#define SCK_PIN     13
#define MISO_PIN    12
#define MOSI_PIN    11
#define CS_PIN      10
#define RESET_PIN   9
#define IRQ_PIN     2

const long frequency = 915E6;
String data_str;
const char *serverURL = "http://192.168.1.17:5000/gps";

// put function declarations here:

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  SPI.begin();
  LoRa.setPins(CS_PIN, RESET_PIN, IRQ_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
    Serial.print("\n Failed to connect to WiFi");
    delay(100);
  }
  Serial.println("\nConnected to the WiFi Network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    int parseSize = LoRa.parsePacket();
    if (parseSize)
    {
      String received = "";
      while (LoRa.available())
      {
        received += (char)LoRa.read();
      }
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

        Serial.print("  Latitude: ");
        Serial.print(latitude, 6);
        Serial.print("  Longitude: ");
        Serial.print(longitude, 6);
        Serial.print("  Altitude: ");
        Serial.print(altitude, 1);
        Serial.print(" m");
        Serial.print("  Speed: ");
        Serial.print(speed, 2);
        Serial.print(" mph");
        Serial.print("  Satellites: ");
        Serial.println(satellites);

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