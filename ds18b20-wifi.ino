#include <OneWire.h>
#include <DallasTemperature.h>

#include <Arduino.h>
#include "pins_arduino.h"
#include <DHT.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#define LED1 D4
#define LED2 D0
bool LEDState=true;
long unsigned int ledFlashRate=60000;
long unsigned int lastFlash;
long unsigned int lastRead;

const char* ssid         = "monsoon";
const char* password     = "S00n3rs!";
const String serverName  = "http://192.168.1.3";
const int httpPort       = 3000;
char httpRequestData[200];

void readDHT();
void sendData();
void flashLED();
void manageWIFI();

// Data wire is plugged TO GPIO 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Number of temperature devices found
int numberOfDevices;

// We'll use this variable to store a found device address
DeviceAddress tempDeviceAddress; 

void setup(){
  // start serial port
  Serial.begin(115200);
  pinMode(LED1, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(LED2, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED1, true);
  digitalWrite(LED2, false);

  manageWIFI();  //This will connect to the WiFi
  
  // Start up the library
  sensors.begin();
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();

  // locate devices on the bus
  Serial.println(2);  
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++){
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }

  lastFlash=millis();
  lastRead=millis();
}

void loop(){ 
  if ( millis() - lastRead > 60000 ) {
    readDHT();
    flashLED();
    lastRead=millis();
  }
    
  if ( millis() - lastFlash > ledFlashRate ) {    
    flashLED();
  }    
}

void manageWIFI(){
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  Serial.println(2);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED1, !digitalRead(LED1));
    digitalWrite(LED2, !digitalRead(LED2));
    delay(500);
  }
  Serial.println("");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());  
}

void flashLED(){
  Serial.println("flash");
  lastFlash=millis();
  digitalWrite(LED1, !digitalRead(LED1));
  digitalWrite(LED2, !digitalRead(LED2));
}

void readDHT() {
  IPAddress myIP=WiFi.localIP();  
  String octet = String(myIP[3]);
  int sLen = octet.length();
  String lastDigit=octet.substring(sLen-1);

  sensors.requestTemperatures();
  float tempArray[numberOfDevices];
  for(int i=0;i<numberOfDevices; i++)
    tempArray[i]=-1;
 
  for(int i=0;i<numberOfDevices; i++){
    if(sensors.getAddress(tempDeviceAddress, i)){
      tempArray[i] = sensors.getTempF(tempDeviceAddress);
      Serial.print(i,DEC);
      Serial.print(": ");
      Serial.println(tempArray[i]); // Converts tempC to Fahrenheit
    }
  }
  sprintf(httpRequestData, "{\"ip\":\"%i.%i.%i.%i\",\"last_digit\":\"%s\",\"room\":\"AC Intake\",\"tempf\":%.2f}", myIP[0], myIP[1], myIP[2], myIP[3], lastDigit, tempArray[0]);
  sendData();
  sprintf(httpRequestData, "{\"ip\":\"%i.%i.%i.%i\",\"last_digit\":\"%s\",\"room\":\"AC Exhaust\",\"tempf\":%.2f}", myIP[0], myIP[1], myIP[2], myIP[3], lastDigit, tempArray[1]);
  sendData();   
}

void sendData(){
  WiFiClient wificlient;
  HTTPClient http;
  String serverPath=serverName+":"+String(httpPort)+"/airqual";
  Serial.println(serverPath + " " + String(httpRequestData));
  
  http.addHeader("Content-Type", "application/json");
  http.begin(wificlient, serverPath.c_str());
  int httpResponseCode = http.POST(httpRequestData);
  if (httpResponseCode != 201){
    manageWIFI();
    ledFlashRate=1000;
  } else {
    ledFlashRate=60000;
  }
  Serial.println("HTTP Response code: " + String(httpResponseCode));
  http.end();
} 

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}
