#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "MRMS ESP32 AP";
const char* password = "19891990";
const char* serverNameTime = "http://192.168.4.1/time";

String timeNow = "                  ";

unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= 1000) {
     // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED ){ 
      timeNow = httpGETRequest(serverNameTime);
     
      Serial.println("Response: " + timeNow);
      
      // save the last HTTP GET Request
      previousMillis = currentMillis;
    }
    else {
      Serial.println("WiFi Disconnected");
    }
  }
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
