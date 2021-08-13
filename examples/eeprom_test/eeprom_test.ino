#include <Wire.h>

struct StrFormat {
  uint32_t startAddress;
  uint8_t length;
};

/*
 * Kirim ini di Serial Monitor
 */

const StrFormat MenuCalibNew{0,19}; // 1
const StrFormat MenuCalibTest{19,20}; 
const StrFormat MenuCalibSave{39,20}; 
const StrFormat MenuFrontSensor{59,16}; 
const StrFormat MenuRearSensor{75,15};
const StrFormat MenuExit{90,11};
const StrFormat MenuMode{101,11};
const StrFormat MenuStep{112,11};
const StrFormat MenuDebug{123,11};
const StrFormat MenuNext{134,11};
const StrFormat MenuBack{145,11}; // 11
const StrFormat MenuMirror{156,10};
const StrFormat MenuNormal{166,10};
const StrFormat MenuInfoMode{176,7};
const StrFormat MenuInfoStep{183,7};
const StrFormat MenuInfoDebug{190,7};
const StrFormat InfoNoDebug{197,8};
const StrFormat InfoByFunc{205,7};
const StrFormat InfoByStep{212,7};
const StrFormat InfoMirror{219,6};
const StrFormat InfoNormal{225,6}; // 21
const StrFormat InfoCountError{231,11}; 
const StrFormat InfoSensorNotDetected{242,23};
const StrFormat InfoSensorWrong{265,12};
const StrFormat InfoSaveSuccess{277,18};
const StrFormat InfoSaveFail{295,15};
const StrFormat InfoCalibTimeout{310,17};
const StrFormat InfoCalibFinish{327,17};
const StrFormat InfoCalibFail{344,15};
const StrFormat InfoLineBlack{359,5};
const StrFormat InfoLineWhite{364,5}; // 31
const StrFormat InfoFF{369,2}; 
const StrFormat InfoBB{371,2};
const StrFormat InfoSdCardError{373,16};
const StrFormat InfoDelimiter{389,21};
const StrFormat FuncBuzzerLed{410,30};
const StrFormat FuncResetTimer{440,17};
const StrFormat FuncPcTimer{457,16};
const StrFormat FuncLineColor{473,18};
const StrFormat FuncSensor{491,15};
const StrFormat FuncError{506,17}; // 41
const StrFormat FuncController{523,22};
const StrFormat FuncFfSpeed{545,19};
const StrFormat FuncBbSpeed{564,19};
const StrFormat FuncLineTrack{583,18};
const StrFormat FuncMotor{601,20};
const StrFormat FuncLine{621,22};
const StrFormat FuncLineT{643,26};
const StrFormat FuncTimeline{669,29};
const StrFormat FuncLeft{698,16};
const StrFormat FuncLeft1{714,17}; // 51
const StrFormat FuncLeft2{731,17};
const StrFormat FuncLeft3{748,17};
const StrFormat FuncLeft4{765,17};
const StrFormat FuncLeft5{782,17};
const StrFormat FuncLeft6{799,17};
const StrFormat FuncLeft7{816,17};
const StrFormat FuncRight{833,17};
const StrFormat FuncRight10{850,19};
const StrFormat FuncRight9{869,18};
const StrFormat FuncRight8{887,18}; // 61
const StrFormat FuncRight7{905,18};
const StrFormat FuncRight6{923,18};
const StrFormat FuncRight5{941,18};
const StrFormat FuncRight4{959,18};
const StrFormat FuncExline{977,24};
const StrFormat FuncExturn{1001,24};
const StrFormat FuncLineDelay{1025,24};
const StrFormat FuncLineFind{1049,23};
const StrFormat FuncLineDLine{1072,33};
const StrFormat FuncLineTLine{1105,33}; // 71
const StrFormat FuncSLine{1138,20};
const StrFormat FuncLostLine{1158,26};
const StrFormat InfoFrontSensor{1184,24};
const StrFormat InfoRearSensor{1208,24};
const StrFormat InfoLineSensor{1232,20};
const StrFormat InfoStep{1252,7};
const StrFormat InfoTimeSpent{1259,13};
const StrFormat InfoYourTime{1272,13};
const StrFormat InfoLineMissing{1285,12};

void setup(){
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  Serial.println(F("My Brain Now!"));
  uint32_t addrtmp = 0;

  #ifdef READ_AND_STORE_EPPROM
  while(addrtmp < 0x1FFFE){
    Serial.print(F("In Addr: "));
    Serial.println(addrtmp, HEX);
    for(uint16_t i=0x00; i<=0xFF; i++){
      byte b = readByteEEPROM(addrtmp + i);
      Serial.print(b, HEX);
      Serial.print('/');
      
      if((i & 0x0F) == 0x0F){
        Serial.println((char) b);
      } else {
        Serial.print((char) b);
        Serial.print(' ');
      }
    }
    addrtmp += 0xFF;
    Serial.print(F("To Addr: "));
    Serial.println(addrtmp, HEX); 
    Serial.println();
  }
  Serial.println(F("Sent me string and i will save it to my brain"));
  #endif
}

uint32_t address = 0;
uint8_t i = 0;
void loop(){
  #ifdef READ_AND_STORE_EEPROM
//  PROGRAM UNTUK MEMBACA SERIAL DAN MENYIMPANNYA MULAI DARI ALAMAT 0x00
  if(Serial.available()){
    char C = (char) Serial.read();
    if(C != '$'){
      if(C != '\n'){
        writeByteEEPROM(address, C);
      }
      
      if(i++ == 0){
        // Serial.print(F("Start writing at address: "));
        Serial.print(address);
        Serial.print(',');
        // Serial.print(F("\tWith length of string: "));
      }
      address++;
    } else {
      Serial.println(i);
      i = 0;
    }
    delay(1);
  }
  #else
// PROGRAM TES MENGAMBIL FORMAT STRING DARI EEPROM DAN MENAMPILKAN KE SERIAL
  Serial.println(getStringFormat(MenuCalibNew));
  Serial.println(getStringFormat(MenuCalibTest));
  Serial.println(getStringFormat(MenuCalibSave));
  Serial.println(getStringFormat(MenuFrontSensor));
  
  String data = getStringFormat(FuncBuzzerLed);
  char info[data.length()] = {0};
  sprintf(info, data.c_str(), 60, 1, 0, 1, 0, 1);
  
  Serial.print(data);
  Serial.print(" -> ");
  Serial.println(info);
  delay(3000);
  #endif
}

String getStringFormat(StrFormat strFormat){
  String result = "";
  for(uint32_t addr = strFormat.startAddress; addr < (strFormat.startAddress + strFormat.length); addr++){
    result += (char) readByteEEPROM(addr);
  }
  return result;
}

void writeByteEEPROM(uint32_t address, byte data){
  if(address > 0x1FFFE){
    Serial.println(F("Maximum address exceed!!!"));
    return; // maximum address
  } 

  uint8_t EEPROM_ADDR = 0x50;
  if(address > 0xFFFF){
    EEPROM_ADDR = 0x51;
    address -= 0xFFFF;
  }

  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((uint8_t) (address >> 8));
  Wire.write((uint8_t) (address & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  
  delay(5);
}

byte readByteEEPROM(uint32_t address){
  if(address > 0x1FFFE){
    Serial.println(F("Maximum address exceed!!!"));
    return 0; // maximum address
  } 

  uint8_t EEPROM_ADDR = 0x50;
  if(address > 0xFFFF){
    EEPROM_ADDR = 0x51;
    address -= 0xFFFF;
  }

  byte data = 0xFF;
  Wire.beginTransmission(EEPROM_ADDR);
  Wire.write((uint8_t) (address >> 8));
  Wire.write((uint8_t) (address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_ADDR, 1);
  if(Wire.available()) data = Wire.read();

  return data;
}