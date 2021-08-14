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

const uint8_t sPin[10] = { s0, s1, s2, s3, s4, s5, s6, s7, s8, s9 };

const int sensorTotal = (sizeof(sPin)/sizeof(sPin[0]));
uint16_t thSensor[sensorTotal] = { 0 };
bool isBlackLine = true;

#define btn1 PB4
#define btn2 PB5
#define led PA8

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
 * Fungsi untuk write data ke EEPROM External melalui komunikasi I2C
 */
void writeByteEEPROM(uint16_t address, byte data){
  if(address > 0x1FFFE){
    Serial.println(F("Maximum address exceed!!!"));
    return; // maximum address
  } 

  Wire.beginTransmission(0x50);
  Wire.write((uint8_t) (address >> 8));
  Wire.write((uint8_t) (address & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  
  delay(5);
}

/*
 * Fungsi untuk mengambil data dari EEPROM External
 */
byte readByteEEPROM(uint16_t address){
  if(address > 0x1FFFE){
    Serial.println(F("Maximum address exceed!!!"));
    return 0; // maximum address
  }

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
uint16_t readValue(uint16_t s){
  uint16_t data;
  uint16_t result = 0x00;

  data = readByteEEPROM(s*2);
  result = data << 8;

  data = readByteEEPROM(s*2+1);
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
    digitalWrite(led, ((millis()-timestart)/500) % 2);
    for(uint8_t i=0; i<sensorTotal; i++){
      realValue = getSensorValue(sPin[i]);
      if(realValue < value[i][0]) value[i][0] = realValue;
      if(realValue > value[i][1]) value[i][1] = realValue;
      thSensor[i] = (value[i][0] + value[i][1]) / 2;
    } 
  }
  digitalWrite(led, 0);
}

/*
 * Fungsi pembacaan button
 */ 
bool readBtn(uint8_t pin){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<5; i++){
    if(digitalRead(pin) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > 4) return true;
  else return false;
}

//******************************** SETUP ********************************
void setup(){
  Serial3.begin(115200);
  Serial.begin(115200);
  delay(10);
  Serial.println(F("Sensor Starting..."));

  Serial.println(F("Read Sensor Threshold..."));
  Wire.begin();
  for(uint8_t i=0; i<sensorTotal; i++){
    pinMode(sPin[i], INPUT);
    thSensor[i] = readValue(i);
    Serial.print('S');
    Serial.print(i);
    Serial.print(": ");
    Serial.println(thSensor[i]);
  }
  pinMode(led, OUTPUT);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  Serial.println(F("Sensor Ready!!!"));
}
//******************************** LOOP  ********************************
unsigned long timing = 0;
bool flag = false;
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
      case '8':
      Serial.println(getSensorValue(8));
      break;
      case '9':
      Serial.println(getSensorValue(9));
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
  
  if(flag == false){
    if(readBtn(btn1)){
      timing = millis();
      digitalWrite(led, 1);
      while(readBtn(btn1)){};
      if((unsigned long) millis()-timing >= 2000){
        flag = true;
        Serial.println(F("Standby Calibration!"));
      } else {
        digitalWrite(led, 0);
      }
    }
    if(readBtn(btn2)){
      Serial.println(F("Read Threshold From EEPROM"));
      for(uint8_t i=0; i<sensorTotal; i++){
        pinMode(sPin[i], INPUT);
        thSensor[i] = readValue(i);
        Serial.print('S');
        Serial.print(i);
        Serial.print(": ");
        Serial.println(thSensor[i]);
      }
      Serial.println(F("Read Threshold Finished"));
      delay(1000);
    }
  } else {
    if(readBtn(btn2)){
      Serial.println(F("Starting Calibration"));
      flag = false;
      calibrateSensor();
      Serial.println(F("Calibration Finished"));
      for(uint8_t i=0; i<sensorTotal; i++){
        saveValue(i, thSensor[i]);
      }
    }
    if(readBtn(btn1)){
      flag = false;
      Serial.println(F("Cancelling Calibration"));
      digitalWrite(led, 0);
    }
  }
}
