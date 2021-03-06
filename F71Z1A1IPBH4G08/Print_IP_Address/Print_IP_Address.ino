#include "ESP8266WiFi.h"
# define LED D4 // Use built-in LED which connected to D4 pin or GPIO 2
// WiFi parameters to be configured
const char* ssid = "sreerupasengupta";
const char* password = "sree1999";

void setup(void)
{ 
  pinMode(LED, OUTPUT);    
  Serial.begin(115200);
  // Connect to WiFi
  WiFi.begin(ssid, password);

  // while wifi not connected yet, print '.'
  // then after it connected, get out of the loop
  while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
  }
  //print a new line, then print WiFi connected and the IP address
  Serial.println("");
  Serial.println("WiFi connected");
  // Print the IP address
  Serial.println(WiFi.localIP());

}
void loop() {
  digitalWrite(LED, HIGH);  // Turn the LED off because the LED is active low
  delay(1000);            // Wait a second
  digitalWrite(LED, LOW);   // Turn the LED on because the LED is active low
  delay(1000);              // Wait a second
}
