#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <Arduino.h>

#define ssid "BKSTAR_T2_02"
#define pass "stemstar"

#define myApi "http://api.thingspeak.com/update?api_key=5F8N9Q7AI0PQZQBH&field1=0&status=ok_very_good"

WiFiClient client;


void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, pass);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.println(".");
  }
  Serial.println("Wifi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  WiFiClient client;
  HTTPClient http;
  // Your IP address with path or Domain name with URL path 
  http.begin(client, myApi);
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  String payload = "{}"; 
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
    Serial.println(payload);
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();
  delay(15*1000);
}
