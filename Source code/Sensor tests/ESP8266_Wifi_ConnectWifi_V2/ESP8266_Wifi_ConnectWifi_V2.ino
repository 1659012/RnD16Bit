#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// WiFi parameters to be configured
//const char* ssid = "HoanLe";
//const char* password = "tu1den9@";
const char* ssid = "FromLabs";
const char* password = "bbf2b4a7d5";
bool isConnected = false;

// Initialize the client library
// HttpClient client;

void setup(void)
{
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);

  // check connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("WiFi not connected.");
  }

  //if connected
  Serial.println("=======================");
  Serial.println("WiFi connected.");

  // Print the IP address
  Serial.println(WiFi.localIP());


}

void loop() {
  isConnected = WiFi.status() == WL_CONNECTED;
  if (isConnected) {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);

    HTTPClient http; //Object of class HTTPClient    
    //http.begin("http://jsonplaceholder.typicode.com/users/1");
    http.begin("https://hoanfunctionapi.azurewebsites.net/api/TestGetObject");   
    int httpCode = http.GET();
    if (httpCode > 0) {
      Serial.println("Get http");
      String rawResult = http.getString();
      //Serial.println(rawResult);

      DynamicJsonDocument doc(1024);
      deserializeJson(doc, rawResult);      

      String name = doc["name"];
      Serial.print("Name: ");
      Serial.println(name);
    }

    http.end();

  }
  delay(5000);
}
