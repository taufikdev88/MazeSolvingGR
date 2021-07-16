/*
 * Gudang Robot
 * 
 * TX RX 3.3 Gnd
 * 
 */
#include <EEPROM.h>

#define s0 PB1
#define s1 PB0
#define s2 PA7
#define s3 PA6
#define s4 PA5
#define s5 PA4
#define s6 PA1
#define s7 PA0

const uint8_t sPin[8] = { PB1, PB0, PA7, PA6, PA5, PA4, PA1, PA0 };
const int sensorTotal = (sizeof(sPin)/sizeof(sPin[0]));
uint16_t thSensor[sensorTotal] = { 0 };
bool isBlackLine = true;

/*
 * Fungsi mengambil nilai analog dari sebuah sensor, dengan fungsi sampling agar
 * nilainya lebih akurat
 */
uint16_t getSensorValue(uint8_t pin, bool sampling = true){
  uint32_t sensorValue = 0;
  if(sampling){
    uint8_t i=0;
    for(i=1; i<=10; i++){
      sensorValue += analogRead(pin);
      delayMicroseconds(10);
    }
    sensorValue = sensorValue / 10;
  } else {
    sensorValue = analogRead(pin);
    delayMicroseconds(10);
  }
  return sensorValue;
}

/*
 * Fungsi menyimpan nilai sebuah sensor ke eeprom
 */
void saveValue(int8_t s, uint16_t v){
  uint16_t address = 0x10;
  uint16_t status;
  
  uint8_t high = v >> 8;
  uint8_t low = v & 0xFF;

  status = EEPROM.write(address+s*2, high);
  //Serial.print("Saving to address ");
  //Serial.print(address+s*2, HEX);
  //Serial.print(": ");
  //Serial.print(high, HEX);
  //Serial.print(" -> ");
  //Serial.println(status);

  status = EEPROM.write(address+s*2+1, low);
  //Serial.print("Saving to address ");
  //Serial.print(address+s*2+1, HEX);
  //Serial.print(": ");
  //Serial.print(low, HEX);
  //Serial.print(" -> ");
  //Serial.println(status);
}

/*
 * Mengambil nilai sebuah sensor dari eeprom
 */
uint16_t readValue(int8_t s){
  uint16_t address = 0x10;
  uint16_t status;
  uint16_t data;
  
  uint16_t result = 0x00;

  status = EEPROM.read(address+s*2, &data);
  //Serial.print("Read address ");
  //Serial.print(address+s*2, HEX);
  //Serial.print(": ");
  //Serial.print(data, HEX);
  //Serial.print(" -> ");
  //Serial.println(status);
  result = data << 8;
  
  status = EEPROM.read(address+s*2+1, &data);
  //Serial.print("Read address ");
  //Serial.print(address+s*2+1, HEX);
  //Serial.print(": ");
  //Serial.print(data, HEX);
  //Serial.print(" -> ");
  //Serial.println(status);
  result = result | data;
  
  return result;
}

/*
 * Kalibrasi sensor selama 10 detik untuk mengambil nilai minimal maksimal
 */
void calibrateSensor(){
  // siapkan variabel
  uint16_t value[sensorTotal][2] = { 0 };
  uint16_t realValue = 0;
  // set nilai variabel awal ke tengah tengah
  for(uint8_t i=0; i<sensorTotal; i++){
    value[i][0] = 1024;
    value[i][1] = 0;
  }
  // jalankan kalibrasi selama 10 detik
  unsigned long timestart = millis();
  while((unsigned long) millis()-timestart <= 10000){
    for(uint8_t i=0; i<sensorTotal; i++){
      realValue = getSensorValue(sPin[i]);
      if(realValue < value[i][0]) value[i][0] = realValue;
      if(realValue > value[i][1]) value[i][1] = realValue;
      thSensor[i] = (value[i][0] + value[i][1]) / 2;
    } 
  }
}

//******************************** SETUP ********************************
void setup(){
  Serial2.begin(115200);
  Serial.begin(115200);

  uint16_t status;
  status = EEPROM.init();

  Serial.print("EEPROM INIT: ");
  Serial.println(status);
  
  for(uint8_t i=0; i<sensorTotal; i++){
    pinMode(sPin[i], INPUT);
    thSensor[i] = readValue(i);
  }
}
//******************************** LOOP  ********************************
void loop() {
  if(Serial.available()){
    char cIn = Serial.read();

    switch(cIn){
      case 'A':
      Serial.print(F("H"));
      for(uint8_t i=0; i<sensorTotal; i++){
        bool biggerThanThreshold = (getSensorValue(sPin[i]) < thSensor[i]);
        if(isBlackLine) biggerThanThreshold = !biggerThanThreshold;
          
        Serial.print(biggerThanThreshold);
      }
      Serial.println('T');
      break;
      case '0':
      Serial.println(getSensorValue(0));
      break;
      case '1':
      Serial.println(getSensorValue(1));
      break;
      case '2':
      Serial.println(getSensorValue(2));
      break;
      case '3':
      Serial.println(getSensorValue(3));
      break;
      case '4':
      Serial.println(getSensorValue(4));
      break;
      case '5':
      Serial.println(getSensorValue(5));
      break;
      case '6':
      Serial.println(getSensorValue(6));
      break;
      case '7':
      Serial.println(getSensorValue(7));
      break;
      case 'Y':
      Serial.print(F("H"));
      for(uint8_t i=0; i<sensorTotal; i++){
        Serial.print(getSensorValue(sPin[i], false));
        Serial.print(',');
      }
      Serial.println('T');
      break;
      case 'Z':
      Serial.print(F("H"));
      for(uint8_t i=0; i<sensorTotal; i++){
        Serial.print(getSensorValue(sPin[i]));
        Serial.print(',');
      }
      Serial.println('T');
      break;
      case 'S':
      for(uint8_t i=0; i<sensorTotal; i++){
        saveValue(i, thSensor[i]);
      }
      Serial.print('D');
      break;
      case 'R':
      for(uint8_t i=0; i<sensorTotal; i++){
        Serial.print(readValue(i));
        Serial.print(',');
      }
      Serial.println();
      break;
      case 'D':
      for(uint8_t i=0; i<sensorTotal; i++){
        Serial.print(thSensor[i]);
        Serial.print(',');
      }
      Serial.println();
      break;
      case 'C':
      calibrateSensor();
      Serial.print('D');
      break;
      case 'B':
      isBlackLine = true;
      Serial.println("Line set to black");
      break;
      case 'W':
      isBlackLine = false;
      Serial.println("Line set to white");
      break;
      case 'T':
      Serial.print(sensorTotal);
      break;
    }
  }
  
  if(Serial2.available()){
    char cIn = Serial2.read();

    switch(cIn){
      case 'A':
      Serial.println("Read all sensor detection");
      Serial2.print(F("H"));
      for(uint8_t i=0; i<sensorTotal; i++){
        bool biggerThanThreshold = (getSensorValue(sPin[i]) < thSensor[i]);
        if(isBlackLine) biggerThanThreshold = !biggerThanThreshold;
          
        Serial2.print(biggerThanThreshold);
        Serial.print(biggerThanThreshold);
      }
      Serial.println();
      Serial2.print('T');
      break;
      case 'S':
      Serial.println("Saving to EEPROM");
      for(uint8_t i=0; i<sensorTotal; i++){
        saveValue(i, thSensor[i]);
      }
      Serial2.print('D');
      break;
      case 'C':
      calibrateSensor();
      Serial2.print('D');
      break;
      case 'B':
      isBlackLine = true;
      Serial.println("Line set to black");
      break;
      case 'W':
      isBlackLine = false;
      Serial.println("Line set to white");
      break;
      case 'T':
      Serial2.print(sensorTotal);
      break;
    }
  }
}
