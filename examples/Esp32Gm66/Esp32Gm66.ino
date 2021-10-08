/*
 * Author       : Taufik Hidayat
 * Create Date  : 9 Oktober 2021 
 * Description  : Program ESP32 untuk menjembatani sensor gm 66 dengan sensor mazesolving
 */
#include <WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
SoftwareSerial testSerial;

//const char* SSID = "RumahNikita";
//const char* PASS = "1WeKKEadyTKlGZ29EgqO5ndZSwjMpSRdcjJImvAltuCsQJE1pHJUclXmWzXrRTd";
const char* SSID = "Unknown";
const char* PASS = "baksoenak";
const char* API_TOPIC = "/mazesolvinggr";

#define MQTT_SERVER "broker.hivemq.com"
#define MQTT_PORT 1883

WiFiClient wifiClient;
PubSubClient client(wifiClient);

unsigned long wifiTiming = millis();
String lastDetectedQr = "";

void reconnect(){
  while(!client.connected()){
    if(WiFi.status() != WL_CONNECTED) break;
    
    Serial.println(F("Attemting MQTT Connection"));
    if(client.connect("robotmaze")){
      Serial.println(F("MQTT Connected"));
    } else {
      Serial.print(F("MQTT Failed, rc="));
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void initWifi(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(SSID, PASS);
  uint8_t cnt = 0;
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print('.');
    digitalWrite(27, !digitalRead(27));
    if(++cnt > 30){
      Serial.println(F("Cannot connect, restarting esp"));
      ESP.restart();
    }
  }
}

void setup() {
  pinMode(27, OUTPUT);
  Serial.begin(115200);
  Serial2.begin(115200);
  testSerial.begin(9600, SWSERIAL_8N1, 4, 2, false, 256);
  initWifi();
  digitalWrite(27, 1);
  Serial.println();
  Serial.println(F("WiFi Connected"));
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  // rutin cek koneksi wifi
  if((WiFi.status() != WL_CONNECTED) && (unsigned long) millis()-wifiTiming >= 5000){
    digitalWrite(27, 0);
    Serial.println(millis());
    Serial.println(F("reconnecting to wifi"));
    initWifi();
    digitalWrite(27, 1);
    wifiTiming = millis();
  }
  // rutin cek koneksi mqtt
  if(!client.connected()){
    reconnect();
  }
  client.loop();
  // rutin untuk menerima serial dari gm66 
  if(testSerial.available()>0){
    String data = "";
    while(testSerial.available() > 0){
      delay(10);
      data += (char) testSerial.read();
      yield();
    }
    lastDetectedQr = data;
    Serial.println(data);
    Serial2.println(data);
  }
  // terima perintah serial untuk command
  if (Serial2.available())
  {
    char c = (char) Serial2.read();
    switch (c){
      case 'K':
        client.publish(API_TOPIC, lastDetectedQr.c_str());
        break;
    }
  }
}
