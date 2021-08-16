/*
 * Gudang Robot
 * 
 * TX RX 3.3 Gnd
 * 
 */
#include <Wire.h>

#define s0 PB1
#define s1 PB0
#define s2 PA7
#define s3 PA6
#define s4 PA5
#define s5 PA4
#define s6 PA3
#define s7 PA2
#define s8 PA1
#define s9 PA0

#define b0 PB4
#define b1 PB5

#define l0 PA8

const uint8_t sPin[10] = { s0, s1, s2, s3, s4, s5, s6, s7, s8, s9 };
const int sensorTotal = (sizeof(sPin)/sizeof(sPin[0]));
uint16_t thSensor[sensorTotal] = { 0 };
bool isBlackLine = true;

unsigned long timing = 0;
bool flag = false;

/*
 * Fungsi membaca tombol dengan sampling
 */
bool readBtn(uint8_t pin){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<5; i++){
    if(digitalRead(pin) == 0) cnt ++;
    delayMicroseconds(10);
  }
  if(cnt > 4) return true;
  return false;
}

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
 * Fungsi simpan byte ke eeprom external
 */
void writeByteEEPROM(uint16_t address, byte data){
  Wire.beginTransmission(0x50);
  Wire.write((uint8_t) (address >> 8));
  Wire.write((uint8_t) (address & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

/*
 * Fungsi ambil byte dari eeprom
 */
byte readByteEEPROM(uint16_t address){
  byte data = 0xFF;
  Wire.beginTransmission(0x50);
  Wire.write((uint8_t) (address >> 8));
  Wire.write((uint8_t) (address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);
  if(Wire.available()) data = Wire.read();
  return data;
}

/*
 * Fungsi menyimpan nilai sebuah sensor ke eeprom
 */
void saveValue(int8_t s, uint16_t v){
  uint8_t high = v >> 8;
  uint8_t low = v & 0xFF;
  writeByteEEPROM(s*2, high);
  writeByteEEPROM(s*2+1, low);
}

/*
 * Mengambil nilai sebuah sensor dari eeprom
 */
uint16_t readValue(int8_t s){
  uint16_t data = 0x0000;
  data = readByteEEPROM(s*2);
  data = data << 8;
  data = data | readByteEEPROM(s*2+1);
  return data;
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
    digitalWrite(l0, (((unsigned long) millis()-timestart) / 500) % 2);
    for(uint8_t i=0; i<sensorTotal; i++){
      realValue = getSensorValue(sPin[i]);
      if(realValue < value[i][0]) value[i][0] = realValue;
      if(realValue > value[i][1]) value[i][1] = realValue;
      thSensor[i] = (value[i][0] + value[i][1]) / 2;
    } 
  }
  digitalWrite(l0, 0);
}

//******************************** SETUP ********************************
void setup(){
  Serial3.begin(115200);
  Serial.begin(115200);
  Wire.begin();
  
  for(uint8_t i=0; i<sensorTotal; i++){
    pinMode(sPin[i], INPUT);
    thSensor[i] = readValue(i);
  }

  pinMode(b0, INPUT_PULLUP);
  pinMode(b1, INPUT_PULLUP);
  pinMode(l0, OUTPUT);
  digitalWrite(l0, 0);
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
      Serial.println(getSensorValue(sPin[0]));
      break;
      case '1':
      Serial.println(getSensorValue(sPin[1]));
      break;
      case '2':
      Serial.println(getSensorValue(sPin[2]));
      break;
      case '3':
      Serial.println(getSensorValue(sPin[3]));
      break;
      case '4':
      Serial.println(getSensorValue(sPin[4]));
      break;
      case '5':
      Serial.println(getSensorValue(sPin[5]));
      break;
      case '6':
      Serial.println(getSensorValue(sPin[6]));
      break;
      case '7':
      Serial.println(getSensorValue(sPin[7]));
      break;
      case '8':
      Serial.println(getSensorValue(sPin[8]));
      break;
      case '9':
      Serial.println(getSensorValue(sPin[9]));
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
  
  if(Serial3.available()){
    char cIn = Serial3.read();

    switch(cIn){
      case 'A':
      Serial.println("Read all sensor detection");
      Serial3.print(F("H"));
      for(uint8_t i=0; i<sensorTotal; i++){
        bool biggerThanThreshold = (getSensorValue(sPin[i]) < thSensor[i]);
        if(isBlackLine) biggerThanThreshold = !biggerThanThreshold;
          
        Serial3.print(biggerThanThreshold);
        Serial.print(biggerThanThreshold);
      }
      Serial.println();
      Serial3.print('T');
      break;
      case 'S':
      Serial.println("Saving to EEPROM");
      for(uint8_t i=0; i<sensorTotal; i++){
        saveValue(i, thSensor[i]);
      }
      Serial3.print('D');
      break;
      case 'C':
      calibrateSensor();
      Serial3.print('D');
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
      Serial3.print(sensorTotal);
      break;
    }
  }

  if(flag == true){
    if(readBtn(b0)){
      flag = false;
      digitalWrite(l0, 0);
    }
    if(readBtn(b1)){
      flag = false;
      calibrateSensor();
      for(uint8_t i=0; i<sensorTotal; i++){
        saveValue(i, thSensor[i]);
      }
    }
  } else {
    if(readBtn(b0)){
      timing = millis();
      digitalWrite(l0, 1);
      while(readBtn(b0));
      if((unsigned long) millis()-timing > 1000){
        flag = true;
      } else {
        digitalWrite(l0, 0);
      }
    }
  }
}
