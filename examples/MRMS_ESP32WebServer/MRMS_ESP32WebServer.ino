#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "MRMS ESP32 AP";
const char* password = "19891990";

AsyncWebServer server(80);

void setup(){
  Serial.begin(115200);
  
  Serial.println("Setting AP…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("IP address: ");
  Serial.println(IP);

  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    char buffer[30];
    sprintf(buffer, "%i ms.\n\r", millis());
    request->send_P(200, "text/plain", buffer);
    Serial.print("Request\n\r");
  });
  
 
  server.begin();
}
 
void loop(){
}
