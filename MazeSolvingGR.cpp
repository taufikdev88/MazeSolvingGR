#include "MazeSolvingGR.h"
/*
 * String format addressing and length
 */
struct StrFormat {
  uint32_t startAddress;
  uint8_t length;
};
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

/*
 * EEPROM ADDRESSING
 */
#define modeaddr 0x10000

/*
 * Pin declaration
 */
#define pwm1 PB0
#define pwm2 PB1
#define pwm3 PB8
#define pwm4 PB9

#define buzz PA8

#define btnMode PA15
#define btnStep PB3
#define btnDebug PB5
#define btnGo PB4

// MPU6050_ADDR    
MPU6050 mpu6050(Wire);

//Servo_ADDR
Adafruit_PWMServoDriver srv = Adafruit_PWMServoDriver();

/*
 * Object declaration
 */
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
HardwareTimer pwmtimer(4); // untuk motor kiri
HardwareTimer pwmtimer2(3); // untuk motor kanan
/*
 * Global variable
 */
#define DELAY_BUZZER 50
#define DELAY_BUTTON 200
// sensor 
#define fsensor Serial1
#define bsensor Serial2
// sensor detection side
#define none -1
#define leftSide 0
#define rightSide 1
// maze config
bool isUseBuzzer = true; // flag menggunakan buzzer atau tidak
bool isTraceForward = true; // flag jalan kedepan atau kebelakang
bool isUseError = true; // flag untuk menggunakan fungsi error jika tidak mendeteksi gariss
bool isErrorDetect = false; // flag jika terdeteksi error
// bool isMirror = false; // flag untuk mode mirroring atau tidak
bool isModeCount = false; // flag untuk fungsi stepcount agar robot tidak bergerak

int8_t runMode = 1; // flag untuk mode1 ,mode2 atau mode3
int8_t nStep = 0; // flag pilihan untuk start dari setpoint tertentu
int8_t mStep = 0; // flag untuk maxStep dan counterStep jika sudah run
uint8_t nFunc = 0; // flag untuk menghitung fungsi yang dijalankan
int8_t debug = 0; // flag untuk pilihan mode debug
#define no_debug 0
#define by_func 1
#define by_step 2

// maze timer
uint16_t errorTime = 500; // flag untuk mengatur lamanya robot menuju error saat tidak mendeteksi garis
uint16_t pcTime = 100; // flag untuk timer percabangan
uint16_t stepDelay = 0; // timer untuk delay dari step menuju fungsi berikutnya
unsigned long startTime = millis(); // timer untuk menyimpan waktu pertama kali robot berjalan
unsigned long errorStart = millis(); // timer untuk menghitung waktu tidak mendeteksi garis
unsigned long tButton = millis(); // timer untuk mendeteksi pemencetan tombol

float Kp = 1;
float Kd = 10;
float dError = 0;
// maze pwm offset
uint8_t leftMotorStartPwmF = 0;
uint8_t rightMotorStartPwmF = 0;
uint8_t leftMotorStartPwmB = 0;
uint8_t rightMotorStartPwmB = 0;
// maze data
bool senData[10] = {0}; // menyimpan nilai sensor
String mazeLog[2]; // variable untuk log terakhir saat menjalankan mode debugging

//IMU
float yaw = 0;

// **************************************************************************************************

/************************************* fungsi yang tidak di tampilkan ke pengguna *******************/
volatile uint32_t counterL = 0;
volatile uint32_t counterR = 0;
unsigned long lastTimeL = 0;
unsigned long lastTimeR = 0;
void pulseCountL(){
  counterL++;
}
void pulseCountR(){
  counterR++;
}
int getRPML(){
  detachInterrupt(digitalPinToInterrupt(PA0));
  int rpm = 60*1000 / (millis()-lastTimeL)*counterL;
  lastTimeL = millis();
  counterL = 0;
  attachInterrupt(digitalPinToInterrupt(PA0),pulseCountL,RISING);
  return rpm;
}
int getRPMR(){
  detachInterrupt(digitalPinToInterrupt(PA1));
  int rpm = 60*1000 / (millis()-lastTimeR)*counterR;
  lastTimeR = millis();
  counterR = 0;
  attachInterrupt(digitalPinToInterrupt(PA1),pulseCountR,RISING);
  return rpm;
}

void turnAngle()
{
  mpu6050.update();
  yaw = mpu6050.getAngleZ();
}

void servo(int pin, uint16_t deg) // servo running control pin 0- 7 
{
  uint16_t pls = deg / 180.0 *450 + 150;
  srv.setPWM(pin, 0, pls);
}

// ----------------------- fungsi independen / tidak bergantung ke fungsi yang lain -- posisi atas
// fungsi pembacaan nilai dari sensor depan dan belakang termasuk dengan parsing dan menyimpan ke variable senData
void readSensor(bool wichSensor){
  HardwareSerial* os;
  bool temp[10] = { 0 };
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  os->write('A');
  bool isSuccess = false;
  unsigned long timewait = millis();
  for(uint8_t i=0; i<12; i++){
    while(!os->available()){
      if((unsigned long) millis()-timewait > 10) break;
    };

    char in = os->read();
    if(i == 0){
      if( in != 'H') break;
    } else if ( i>0 && i<11){
      if(!isDigit(in)) break;
      temp[i-1] = (in == '0' ? 0 : 1);
    } else if(i == 11){
      if( in != 'T' ) break;
      for(i=0; i<10; i++){
        senData[i] = temp[i];
      }
      i = 11; // keluar loop
      isSuccess = true;
    }
  }
  // jika pembacaan timeout, set senData ke 0 semua
  if(!isSuccess){
    for(uint8_t i=0; i<10; i++){
      senData[i] = 0;
    }
  }
}
// fungsi pembacaan tombol
#define isBtn1() readBtn(btnMode)
#define isBtn2() readBtn(btnStep)
#define isBtn3() readBtn(btnDebug)
#define isBtn4() readBtn(btnGo)
#define isBtnMode() readBtn(btnMode)
#define isBtnStep() readBtn(btnStep)
#define isBtnDebug() readBtn(btnDebug)
#define isBtnGo() readBtn(btnGo)
bool readBtn(uint8_t btn){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<7; i++){
    if(digitalRead(btn) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > 4) return true;
  return false;
}
// fungsi penulisan byte ke EEPROM External
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
// fungsi pembacaan byte dari EEPROM External
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
// fungsi pengambilan String Format dari EEPROM
String getStringFormat(StrFormat strFormat){
  String result = "";
  for(uint32_t addr = strFormat.startAddress; addr < (strFormat.startAddress + strFormat.length); addr++){
    result += (char) readByteEEPROM(addr);
  }
  return result;
}
// fungsi mengambil last mode yang dijalankan oleh robot
// disimpan di address pertama EEPROM external ke 2
const char mode_n[3] = "md";
void getLastMode(){
  runMode = readByteEEPROM(modeaddr);
  if(runMode < 1 || runMode > 3){
    runMode = 1;
  }
}
// fungsi menyimpan last mode yang dijalankan oleh robot
void setLastMode(){
  writeByteEEPROM(modeaddr, runMode);
}
// fungsi menghitung berapa jumlah setpoint yang diatur dari masing masing mode
void countSetpoint(){
  mStep = -1;
  nStep = 0;
  isModeCount = true;
  switch (runMode){
    case 1: mode1(); break;
    case 2: mode2(); break;
    case 3: mode3(); break;
  }
  isModeCount = false;
}
// fungsi untuk menampilkan log ke layar saat mode debug
void logPrint(String newLog){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(runMode); display.print(F(" / "));
  // display.print(runMode); display.print(F(" / ")); display.print(isMirror ? getText(31):getText(32)); display.print(F(" / ")); 
  switch (debug){
    case no_debug: display.println(getStringFormat(InfoNoDebug)); break;
    case by_func: display.println(getStringFormat(InfoByFunc)); break;
    case by_step: display.println(getStringFormat(InfoByStep)); break;
  }
  display.println(getStringFormat(InfoDelimiter));
  display.setCursor(0,16);
  mazeLog[0] = mazeLog[1];
  mazeLog[1] = newLog;
  display.println(mazeLog[0]);
  display.println(mazeLog[1]);
  display.display();
}
// fungsi khusus untuk menghapus seluruh layar dan menampilkan pesan dari posisi 0,0
void printDisplayHome(String text){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(text);
  display.display();
}
// fungsi merubah array senData ke penampung berupa uint8_t agar mudah untuk pengolahan nilai sensor dalam bentuk biner
void senData2Bin(uint16_t *line){
  if(senData[0]) *line = *line | 0b1000000000;
  if(senData[1]) *line = *line | 0b0100000000;
  if(senData[2]) *line = *line | 0b0010000000;
  if(senData[3]) *line = *line | 0b0001000000;
  if(senData[4]) *line = *line | 0b0000100000;
  if(senData[5]) *line = *line | 0b0000010000;
  if(senData[6]) *line = *line | 0b0000001000;
  if(senData[7]) *line = *line | 0b0000000100;
  if(senData[8]) *line = *line | 0b0000000010;
  if(senData[9]) *line = *line | 0b0000000001;
}
// fungsi untuk menjalankan motor kanan kiri berdasarkan skala pwm speed 0-20 
#define PWM_RESOLUTION 3600
void kinematik(int8_t leftSpeed, int8_t rightSpeed){
  float fSpeed = 0.0;
  if(leftSpeed < 0){
    fSpeed = (float) leftSpeed / -20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + leftMotorStartPwmB), 0, PWM_RESOLUTION);
    pwmWrite(pwm1, 0);
    pwmWrite(pwm2, fSpeed);
  } else {
    fSpeed = (float) leftSpeed / 20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + leftMotorStartPwmF), 0, PWM_RESOLUTION);
    pwmWrite(pwm1, fSpeed);
    pwmWrite(pwm2, 0);
  }
  if(rightSpeed < 0){
    fSpeed = (float) rightSpeed / -20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + rightMotorStartPwmB), 0, PWM_RESOLUTION);
    pwmWrite(pwm3, 0);
    pwmWrite(pwm4, fSpeed);
  } else {
    fSpeed = (float) rightSpeed / 20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + rightMotorStartPwmF), 0, PWM_RESOLUTION);
    pwmWrite(pwm3, fSpeed);
    pwmWrite(pwm4, 0);
  }
}
void motor_f(int8_t leftSpeed, int8_t rightSpeed, uint16_t runTime = 0){
  leftSpeed = constrain(leftSpeed, -20, 20);
  rightSpeed = constrain(rightSpeed, -20, 20);
  if(runTime > 0){
    unsigned long timeStart = millis();
    while((unsigned long) millis()-timeStart <= runTime){
      kinematik(leftSpeed, rightSpeed);
    }
  } else {
    kinematik(leftSpeed, rightSpeed);
  }
  if(runTime != 0){
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
    analogWrite(pwm3, 0);
    analogWrite(pwm4, 0);
  }
}
// -------------------------- fungsi dependen ----- membutuhkan fungsi independen agar bisa jalan
// fungsi menampilkan menu pertama standby 
void displayMenu(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(getStringFormat(MenuInfoMode)); display.println(runMode);
  display.print(getStringFormat(MenuInfoStep)); display.println(nStep);
  display.print(getStringFormat(MenuInfoDebug)); 
  switch(debug){
    case no_debug: display.println(getStringFormat(InfoNoDebug)); break;
    case by_func: display.println(getStringFormat(InfoByFunc)); break;
    case by_step: display.println(getStringFormat(InfoByStep)); break;
    default: debug = no_debug; break;
  }

  display.println(getStringFormat(MenuNext));
  char d[getStringFormat(InfoFrontSensor).length() + 10] = {0};
  sprintf(d,"rpm_l:%d r:%d", counterL, counterR);
  display.println(d);

  display.setCursor(0,48);
  readSensor(ff);
  sprintf(d,getStringFormat(InfoFrontSensor).c_str(), senData[0], senData[1], senData[2], senData[3], senData[4], senData[5], senData[6], senData[7], senData[8], senData[9]);
  display.println(d);
  readSensor(bb);
  sprintf(d,getStringFormat(InfoRearSensor).c_str(), senData[9], senData[8], senData[7], senData[6], senData[5], senData[4], senData[3], senData[2], senData[1], senData[0]);
  display.println(d);
  display.display();
}
// fungsi yang dijalankan saat robot menemukan error
void errorRaised(){
  while(1){    
    motor_f(0,0,0);
    digitalWrite(buzz, 1);
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    uint16_t line = 0;
    senData2Bin(&line);

    delay(DELAY_BUZZER);
    digitalWrite(buzz, 0);
    delay(DELAY_BUZZER);
    if(line != 0) break;
  }
  logPrint(F("run"));
}
// fungsi yang menjalankan motor sesuai PID untuk sensor garis
void controllerRun(uint16_t line, int8_t speed, bool useError = true){
  int8_t error = 0;
  int8_t pwm = 0;
  
  switch(line){
    // garis 2 sensor
    case 0b1000000000: error = -27; isErrorDetect = false; break;
    case 0b1100000000: error = -24; isErrorDetect = false; break;
    case 0b0100000000: error = -21; isErrorDetect = false; break;
    case 0b0110000000: error = -19; isErrorDetect = false; break;
    case 0b0010000000: error = -16; isErrorDetect = false; break;
    case 0b0011000000: error = -12; isErrorDetect = false; break;
    case 0b0001000000: error = -10; isErrorDetect = false; break;
    case 0b0001100000: error = -5; isErrorDetect = false; break;
    case 0b0000100000: error = -2; isErrorDetect = false; break;
    case 0b0000010000: error = 2; isErrorDetect = false; break;
    case 0b0000011000: error = 5; isErrorDetect = false; break;
    case 0b0000001000: error = 10; isErrorDetect = false; break;
    case 0b0000001100: error = 12; isErrorDetect = false; break;
    case 0b0000000100: error = 16; isErrorDetect = false; break;
    case 0b0000000110: error = 19; isErrorDetect = false; break;
    case 0b0000000010: error = 21; isErrorDetect = false; break;
    case 0b0000000011: error = 24; isErrorDetect = false; break;
    case 0b0000000001: error = 27; isErrorDetect = false; break;
    
    // garis 3 sensor
    case 0b1110000000: error = -21; isErrorDetect = false; break;
    case 0b0111000000: error = -14; isErrorDetect = false; break;
    case 0b0011100000: error = -7; isErrorDetect = false; break;
    case 0b0001110000: error = -1; isErrorDetect = false; break;
    case 0b0000111000: error = 1; isErrorDetect = false; break;
    case 0b0000011100: error = 7; isErrorDetect = false; break;
    case 0b0000001110: error = 14; isErrorDetect = false; break;
    case 0b0000000111: error = 21; isErrorDetect = false; break;

    // garis 4 sensor
    case 0b1111000000: error = -36; isErrorDetect = false; break; 
    case 0b0111100000: error = -28; isErrorDetect = false; break;
    case 0b0000011110: error = 28; isErrorDetect = false; break;
    case 0b0000001111: error = 36; isErrorDetect = false; break;

    // normal 
    case 0b0000110000: error = 0; isErrorDetect = false; break;
    
    // error di sensor lain tidak begitu parah
    case 0b0000110001:
    case 0b0000110011:
    case 0b0000110100:
    case 0b0000110010:
    case 0b0000110110:
    case 0b0010110000:
    case 0b0100110000:
    case 0b0110110000:
    case 0b1100110000:
    case 0b1000110000:
    // error di sensor lain tp parah
    case 0b0011110000:
    case 0b0000111100:
    // error tp imbang di kanan kiri sensor
    case 0b0001111000:
    case 0b0011111100:
    case 0b0111111110:
    case 0b0100110010:
    case 0b1000110001:
    case 0b1100110011:
    case 0b0010110100:
    case 0b0110110110:
    // invers error tp imbang di kanan kiri sensor
    case 0b1111001111:
    case 0b1110000111:
    case 0b1100000011:
    case 0b1000000001:
    case 0b0111001110:
    case 0b0011001100:
    case 0b0001001000:
    case 0b0010000100: error = 0; isErrorDetect = false; break;
    
    // error sensor kondisi tengah
    case 0b0000100001:
    case 0b0000100011:
    case 0b0000100100:
    case 0b0000100010:
    case 0b0000100110: error = -1; isErrorDetect = false; break;
    case 0b0010010000:
    case 0b0100010000:
    case 0b1100010000:
    case 0b1000010000:
    case 0b0110010000: error = 1; isErrorDetect = false; break;

    // error sensor kondisi samping medium
    case 0b0001100001:
    case 0b0001100011:
    case 0b0001100100:
    case 0b0001100010:
    case 0b0001100110: error = -3; isErrorDetect = false; break;
    case 0b1000011000:
    case 0b1100011000:
    case 0b0010011000:
    case 0b0100011000:
    case 0b0110011000: error = 3; isErrorDetect = false; break;

    // error sensor kondisi samping high
    // case 0b00100010:
    // case 0b00100001:
    // case 0b00100011: error = -5; isErrorDetect = false; break;
    // case 0b01000100:
    // case 0b10000100:
    // case 0b11000100: error = 5; isErrorDetect = false; break;

    // garis di semua line
    case 0b1111111111: error = 0; isErrorDetect = false; break;

    case 0b0000000000: 
    error = 0;
    if(!isErrorDetect){
      logPrint(getStringFormat(InfoLineMissing));
      errorStart = millis();
      isErrorDetect = true;
    }
    break;
  }

  if(useError && isErrorDetect && (unsigned long) millis()-errorStart >= errorTime && isUseError){
    errorRaised();
  }

  pwm = Kp*error + Kd*(error-dError);
  dError = error;

  int8_t s1 = speed+pwm;
  int8_t s2 = speed-pwm;
  
  if(isTraceForward) motor_f(s1, s2, 0);
  else motor_f(-s2, -s1, 0);
}
// fungsi yang digunakan untuk brake time setelah fungsi fungsi odometri
void backBrakeTimeF(uint8_t speed, int16_t brakeTime){
  // brakeTime
  if(brakeTime > 0){
    if(isTraceForward){
      motor_f(-speed,-speed, brakeTime);
    } else {
      motor_f(speed, speed, brakeTime);
    }
  } else 
  if(brakeTime < 0){
    unsigned long timeStart = millis();
    while((unsigned long) millis()-timeStart <= abs(brakeTime)){
      if(isTraceForward) readSensor(ff);
      else readSensor(bb);

      uint16_t line = 0x00;
      senData2Bin(&line);

      controllerRun(line, speed);
    }
  }
  motor_f(0,0,0);
}
// fungsi pendeteksian percabangan berdasarkan metode dan arah yang diberikan
void runAndDetect(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime){  
  // siapkan variable flag untuk mode pp
  unsigned long timeDetect1 = millis();
  unsigned long timeDetect2 = millis();
  unsigned long timeStart = millis();
  
  bool isTimeDetect = false;
  int8_t side = -1;
  
  bool crossDetected = false;
  isErrorDetect = false;
  // main loop 
  while(!crossDetected){
    // ambil data sensor
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    if(actionTime != (uint16_t) -1){
      if((unsigned long) millis()-timeStart >= actionTime) break;
    }
    
    // deteksi percabangan sesuai metode
    switch(method){
      case pp0:
      if(senData[0]){
        if(!isTimeDetect){
          isTimeDetect = true;
          side = leftSide;
          timeDetect1 = millis();
        } else if(side != leftSide && isTimeDetect){
          timeDetect2 = millis();
          if((timeDetect2 - timeDetect1) < pcTime){
            crossDetected = true;
          } else {
            isTimeDetect = false;
            side = none;
          }
        }
      }
      if(senData[9]){
        if(!isTimeDetect){
          isTimeDetect = true;
          side = rightSide;
          timeDetect1 = millis();
        } else if(side != rightSide && isTimeDetect){
          timeDetect2 = millis();
          if((timeDetect2 - timeDetect1) < pcTime){
            crossDetected = true;
          } else {
            isTimeDetect = false;
            side = none;
          }
        }
      }
      break;
      case pp1:
      if(senData[0] && senData[1]){
        if(!isTimeDetect){
          isTimeDetect = true;
          side = leftSide;
          timeDetect1 = millis();
        } else if(side != leftSide && isTimeDetect){
          timeDetect2 = millis();
          if((timeDetect2 - timeDetect1) < pcTime){
            crossDetected = true;
          } else {
            isTimeDetect = false;
            side = none;
          }
        }
      }
      if(senData[8] && senData[9]){
        if(!isTimeDetect){
          isTimeDetect = true;
          side = rightSide;
          timeDetect1 = millis();
        } else if(side != rightSide && isTimeDetect){
          timeDetect2 = millis();
          if((timeDetect2 - timeDetect1) < pcTime){
            crossDetected = true;
          } else {
            isTimeDetect = false;
            side = none;
          }
        }
      }
      break;
      case pp2:
      // penggunaan variable side dijadikan penghitung nilai deteksi sensor
      side = 0;
      if(senData[0]) ++side;
      if(senData[1]) ++side;
      if(senData[2]) ++side;
      if(senData[7]) ++side;
      if(senData[8]) ++side;
      if(senData[9]) ++side;
      if(side >= 5) crossDetected = true;
      break;
      case pp3:
      side = 0;
      if(senData[0]) ++side;
      if(senData[1]) ++side;
      if(senData[2]) ++side;
      if(senData[7]) ++side;
      if(senData[8]) ++side;
      if(senData[9]) ++side;
      if(side >= 6) crossDetected = true;
      break;
      case tt1:
      if((dir == fl || dir == ll || dir == sl) && senData[0]){
        crossDetected = true;
      } else 
      if((dir == fr || dir == rr || dir == sr) && senData[9]){
        crossDetected = true;
      }
      break;
      case tt2:
      if((dir == fl || dir == ll || dir == sl) && senData[0] && senData[1]){
        crossDetected = true;
      } else 
      if((dir == fr || dir == rr || dir == sr) && senData[8] && senData[9]){
        crossDetected = true;
      }
      break;
      case tt3:
      if((dir == fl || dir == ll || dir == sl) && senData[0] && senData[1] && senData[2]){
        crossDetected = true;
      } else 
      if((dir == fr || dir == rr || dir == sr) && senData[7] && senData[8] && senData[9]){
        crossDetected = true;
      }
      break;
    }
    if(crossDetected) continue;
    
    uint16_t line = 0;
    senData2Bin(&line);

    /*
    switch(method){
      case pp0:
      case tt1:
      if(senData[1]) line = line | 0b01000000;
      if(senData[6]) line = line | 0b00000010;
      
      case pp1:
      case tt2:
      if(senData[2]) line = line | 0b00100000;
      if(senData[5]) line = line | 0b00000100;

      case pp2:
      case pp3:
      case tt3:
      if(senData[3]) line = line | 0b00010000;
      if(senData[4]) line = line | 0b00001000;
      break;
    }
    */
    controllerRun(line, speed);
  }
  
  switch(dir){
    case ss:
    case sl:
    case sr:
    motor_f(0,0,0);
    break;
    
    case ff: 
    case fl:
    case fr:
    case ll:
    case rr:
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    while(senData[0] || senData[9]){
      if(isTraceForward){ 
        readSensor(ff);
        motor_f(speed,speed,0);
      } else {
        readSensor(bb);
        motor_f(-speed,-speed,0);
      }
    }
    motor_f(0,0,0);
    break;
  } 

  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  backBrakeTimeF(speed, brakeTime);

  // turning 
  crossDetected = false;
  switch(dir){
    case ll:
    while(!crossDetected || (!senData[4] && !senData[5])){
      if(isTraceForward){
        motor_f(-speed,speed,0);
        readSensor(ff);
      } else {
        motor_f(speed,-speed,0);
        readSensor(bb);
      }
      if(senData[1]) crossDetected = true;
    }
    break;
    case rr:
    while(!crossDetected || (!senData[4] && !senData[5])){
      if(isTraceForward){
        motor_f(speed,-speed,0);
        readSensor(ff);
      } else {
        motor_f(-speed,speed,0);
        readSensor(bb);
      }
      if(senData[6]) crossDetected = true;
    }
    break; 
  }
  motor_f(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void line_f(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime){
  speed = constrain(speed, -20, 20);
  brakeTime = constrain(brakeTime, -30000, 30000);

  runAndDetect(method, dir, speed, brakeTime, -1);
}
// sudah
void linet_f(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime){
  speed = constrain(speed, -20, 20);
  brakeTime = constrain(brakeTime, -30000, 30000);

  runAndDetect(method, dir, speed, brakeTime, actionTime);
}
// sudah
void linedelay_f(uint8_t speed, uint16_t runTime, int16_t backBrakeTime){
  speed = constrain(speed, 0, 20);
  runTime = constrain(runTime, 0, 65000);
  backBrakeTime = constrain(backBrakeTime, -255, 255);

  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart <= runTime){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    uint16_t line = 0x00;
    senData2Bin(&line);

    controllerRun(line, speed);
  }
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  if(backBrakeTime < 0){
    if(isTraceForward) motor_f(speed, speed, backBrakeTime);
    else motor_f(-speed, -speed, backBrakeTime);
  } else if(backBrakeTime > 0){
    if(isTraceForward) motor_f(-speed, -speed, backBrakeTime);
    else motor_f(speed, speed, backBrakeTime);
  }
  motor_f(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  digitalWrite(buzz, 0);
  digitalWrite(LED_BUILTIN, 0);
}
// sudah
void turn(bool dir, uint8_t sensor, uint8_t speed, uint8_t backBrakeTime){
  speed = constrain(speed, 0, 20);
  
  // baca sensor pertama
  if(isTraceForward) readSensor(ff);
  else readSensor(bb);
  // turning
  while(!senData[sensor]){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    
    switch(dir){
      case leftSide:
      motor_f(-speed, speed, 0);
      break;
      case rightSide:
      motor_f(speed, -speed, 0);
      break;
    } 
  }
  // stop
  motor_f(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  // reverse turning
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart<backBrakeTime){
    switch(dir){
      case leftSide:
      motor_f(speed, -speed, 0);
      break;
      case rightSide:
      motor_f(-speed, speed, 0);
      break;
    }
  }
  // stop
  motor_f(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
/****************************** Batas fungsi yang tidak ditampilkan ke pengguna ***************************/

/****************************** Fungsi yang bisa dijalankan oleh pengguna *********************************/
// fungsi untuk menandai setpoint mulai setpoint 0 -> sampai seterusnya
void step(){
  ++mStep;
  if(mStep < nStep) return; 
  delay(stepDelay);

  if(debug == by_step) {
    char d[getStringFormat(InfoStep).length() + 2] = {0};
    sprintf(d, getStringFormat(InfoStep).c_str(),mStep);
    logPrint(d);
    while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);
  }
}
// sudah
void setStepDelay(uint16_t time){
  ++nFunc;
  stepDelay = time;
}
// sudah
void buzzerled(bool useBuzzer, bool useLED, uint8_t numberOfTimes, uint16_t interval, uint16_t customOffTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return; // hanya berlaku untuk fungsi yang tidak berfungsi untuk mengatur config maze
  
  if(debug != no_debug && !isErrorDetect){
    char d[getStringFormat(FuncBuzzerLed).length()+15] = {0};
    sprintf(d, getStringFormat(FuncBuzzerLed).c_str(),nFunc,useBuzzer,useLED,numberOfTimes,interval,customOffTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  bool state = true;
  interval = constrain(interval, 0, 65000);
  customOffTime = constrain(customOffTime, 0, 65000);
  for(uint16_t i=0; i<(uint16_t) numberOfTimes*2; i++){
    if(state){
      if(useBuzzer) digitalWrite(buzz, 1);
      if(useLED) digitalWrite(LED_BUILTIN, 1);
      delay(interval);
    } else {
      digitalWrite(LED_BUILTIN, 0);
      digitalWrite(buzz, 0);
      if(i == numberOfTimes*2 - 1 && numberOfTimes != 1) break;
      if(customOffTime != 0){
        delay(customOffTime);
      } else {
        delay(interval);
      }
    }
    state = !state;
  }
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(buzz, 0);
}
// sudah
void start(bool useBuzzer, uint16_t pc){
  if(isModeCount) return;
  pc = constrain(pc, 0, 65000);
  isUseBuzzer = useBuzzer;
  pcTime = pc;
}
// sudah
void resettimer(){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncResetTimer).length() + 2] = {0};
    sprintf(d,getStringFormat(FuncResetTimer).c_str(),nFunc);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  startTime = millis();
}
// sudah 
void pctimer(uint16_t pc){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncPcTimer).length() + 2] = {0};
    sprintf(d,getStringFormat(FuncPcTimer).c_str(),pc);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  pc = constrain(pc, 0, 65000);
  pcTime = pc;
}
// sudah
void linecolor(bool color){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLineColor).length() + 5] = {0};
    sprintf(d,getStringFormat(FuncLineColor).c_str(),nFunc,(color == black ? getStringFormat(InfoLineBlack).c_str():getStringFormat(InfoLineWhite).c_str()));
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  if(color == black){
    fsensor.print('B');
    bsensor.print('B');
  } else {
    fsensor.print('W');
    bsensor.print('W');
  }
}
// sudah
void sensor(bool dir){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncSensor).length() + 5] = {0};
    sprintf(d,getStringFormat(FuncSensor).c_str(),nFunc,(dir==ff?getStringFormat(InfoFF).c_str():getStringFormat(InfoBB).c_str()));
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  if(dir == ff){
    isTraceForward = true;
  } else {
    isTraceForward = false;
  }
}
// sudah
void error(bool useError, uint16_t time){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncError).length() + 10] = {0};
    sprintf(d,getStringFormat(FuncError).c_str(),nFunc,useError,time);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  isUseError = useError;
  errorTime = time;
}
// sudah
void controller(float customKp, float customKd){
  if(isModeCount) return;
  ++nFunc;

  Kp = constrain(customKp, 0.0, 100.0);
  Kd = constrain(customKd, 0.0, 100.0);
  
  if(debug != no_debug){
    char p1[9] = {0};
    char p2[9] = {0};
    dtostrf(customKp,2,5,p1);
    dtostrf(customKd,2,5,p2);
    char d[getStringFormat(FuncController).length()+10] = {0};
    sprintf(d,getStringFormat(FuncController).c_str(),nFunc,p1,p2);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}
}
// sudah
void ffspeed(uint8_t l, uint8_t r){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncFfSpeed).length() + 10] = {0};
    sprintf(d,getStringFormat(FuncFfSpeed).c_str(),nFunc,l,r);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  leftMotorStartPwmF = l;
  rightMotorStartPwmF = r;
}
// sudah
void bbspeed(uint8_t l, uint8_t r){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncBbSpeed).length() + 10] = {0};
    sprintf(d,getStringFormat(FuncBbSpeed).c_str(),nFunc,l,r);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  leftMotorStartPwmB = l;
  rightMotorStartPwmB = r;
}
// sudah
void motor(int8_t leftSpeed, int8_t rightSpeed, uint16_t runTime = 0){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;

  if(debug != no_debug){
    char d[getStringFormat(FuncMotor).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncMotor).c_str(),nFunc,leftSpeed,rightSpeed,runTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}
  
  motor_f(leftSpeed, rightSpeed, runTime);
}
// sudah
void line(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLine).length() + 20] = {0};
    sprintf(d,getStringFormat(FuncLine).c_str(),nFunc,method,dir,speed,brakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  line_f(method, dir, speed, brakeTime);
}
// sudah
void linet(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLineT).length() + 30] = {0};
    sprintf(d,getStringFormat(FuncLineT).c_str(),nFunc,method,dir,speed,brakeTime,actionTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  linet_f(method,dir,speed,brakeTime,actionTime);
}
// sudah
void timeline(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t totalActionTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncTimeline).length() + 30] = {0};
    sprintf(d,getStringFormat(FuncTimeline).c_str(),nFunc,method,dir,speed,brakeTime,totalActionTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  speed = constrain(speed, -20, 20);
  brakeTime = constrain(brakeTime, -30000, 30000);

  while((unsigned long) millis()-startTime < totalActionTime);
  runAndDetect(method, dir, speed, brakeTime, -1);
}
// sudah
void left(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;

  if(debug != no_debug){
    char d[getStringFormat(FuncLeft).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 1, speed, backBrakeTime);
}
void left1(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;

  if(debug != no_debug){
    char d[getStringFormat(FuncLeft1).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft1).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 0, speed, backBrakeTime);
}
void left2(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;

  if(debug != no_debug){
    char d[getStringFormat(FuncLeft2).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft2).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 1, speed, backBrakeTime);
}
void left3(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLeft3).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft3).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 2, speed, backBrakeTime);
}
void left4(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLeft4).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft4).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 3, speed, backBrakeTime);
}
void left5(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLeft5).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft5).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 4, speed, backBrakeTime);
}
void left6(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLeft6).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft6).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 5, speed, backBrakeTime);
}
void left7(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLeft7).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLeft7).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 6, speed, backBrakeTime);
}
// sudah
void right(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 6, speed, backBrakeTime);
}
void right10(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight10).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight10).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 9, speed, backBrakeTime);
}
void right9(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight9).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight9).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 8, speed, backBrakeTime);
}
void right8(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight8).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight8).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 7, speed, backBrakeTime);
}
void right7(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight7).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight7).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 6, speed, backBrakeTime);
}
void right6(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight6).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight6).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 5, speed, backBrakeTime);
}
void right5(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight5).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight5).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 4, speed, backBrakeTime);
}
void right4(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncRight4).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncRight4).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 3, speed, backBrakeTime);
}
// sudah
void exline(int8_t leftMotorSpeed, int8_t rightMotorSpeed, uint8_t sensor, int16_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncExline).length() + 30] = {0};
    sprintf(d,getStringFormat(FuncExline).c_str(),nFunc,leftMotorSpeed,rightMotorSpeed,sensor,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  backBrakeTime = constrain(backBrakeTime, -255, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, -20, 20);
  rightMotorSpeed = constrain(rightMotorSpeed, -20, 20);
  bool lineDetected = false;
  
  while(!lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint16_t line = 0x00;
    senData2Bin(&line);  
    if((line & sensor) == sensor) lineDetected = true;
    if(isTraceForward){
      motor_f(leftMotorSpeed, rightMotorSpeed, 0);
    } else {
      motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
    }
  }
  while(lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint16_t line = 0x00;
    senData2Bin(&line);  
    if((line & sensor) == 0x00) lineDetected = false;  
    if(isTraceForward){
      motor_f(leftMotorSpeed, rightMotorSpeed, 0);
    } else {
      motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
    }
  }
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart<abs(backBrakeTime)){
    if(isTraceForward){
      if(backBrakeTime < 0){
        motor_f(leftMotorSpeed, rightMotorSpeed, 0);
      } else {
        motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
      }
    } else {
      if(backBrakeTime < 0){
        motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
      } else {
        motor_f(leftMotorSpeed, rightMotorSpeed, 0);
      }
    }
  }
  motor_f(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void exturn(int8_t leftMotorSpeed, int8_t rightMotorSpeed, uint8_t sensor, int16_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncExturn).length() + 20] = {0};
    sprintf(d,getStringFormat(FuncExturn).c_str(),leftMotorSpeed,rightMotorSpeed,sensor,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  backBrakeTime = constrain(backBrakeTime, -255, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, -20, 20);
  rightMotorSpeed = constrain(rightMotorSpeed, -20, 20);
  bool lineDetected = false;
  
  while(!lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint16_t line = 0x00;
    senData2Bin(&line);
    if((line & sensor) == sensor) lineDetected = true;
    
    if(isTraceForward){
      motor_f(leftMotorSpeed, rightMotorSpeed, 0);
    } else {
      motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
    }
  }
  motor_f(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart<abs(backBrakeTime)){
    if(isTraceForward){
      if(backBrakeTime < 0){
        motor_f(leftMotorSpeed, rightMotorSpeed, 0);
      } else {
        motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
      }
    } else {
      if(backBrakeTime < 0){
        motor_f(-leftMotorSpeed, -rightMotorSpeed, 0);
      } else {
        motor_f(leftMotorSpeed, rightMotorSpeed, 0);
      }
    }
  }
  motor_f(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void linedelay(uint8_t speed, uint16_t runTime, int16_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLineDelay).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncLineDelay).c_str(),nFunc,speed,runTime,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  linedelay_f(speed, runTime, backBrakeTime);
}
// sudah
void linefind(int8_t leftSpeed, int8_t rightSpeed, uint16_t timeToDisregardLine){   
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    char d[getStringFormat(FuncLineFind).length() + 25] = {0};
    sprintf(d,getStringFormat(FuncLineFind).c_str(),nFunc,leftSpeed,rightSpeed,timeToDisregardLine);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  leftSpeed = constrain(leftSpeed, -20, 20);
  rightSpeed = constrain(rightSpeed, -20, 20);
  timeToDisregardLine = constrain(timeToDisregardLine, 0, 65000);
  bool isFound = false;

  unsigned long timeStart = millis();
  while(!isFound){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    motor_f(leftSpeed, rightSpeed, 0);
    for(int8_t i=0; i<10; i++){
      if(senData[i]){
        if((unsigned long) millis()-timeStart > timeToDisregardLine){
          isFound = true;
        }
      }
    }
  }
  motor_f(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1);
    delay(DELAY_BUZZER);
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0);
  }
}
// sudah
void linedline(uint16_t runTime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t brakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;

  if(debug != no_debug){
    char d[getStringFormat(FuncLineDLine).length() + 30] = {0};
    sprintf(d,getStringFormat(FuncLineDLine).c_str(),nFunc,runTime,startSpeed,method,dir,endSpeed,brakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  linedelay_f(startSpeed, runTime, 0);
  line_f(method,dir,endSpeed,brakeTime);
}
// sudah
void linetline(uint16_t runTime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t brakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;

  if(debug != no_debug){
    char d[getStringFormat(FuncLineTLine).length() + 30] = {0};
    sprintf(d,getStringFormat(FuncLineTLine).c_str(),nFunc,runTime,startSpeed,method,dir,endSpeed,brakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  linet_f(method,dir,startSpeed,0,runTime);
  line_f(method,dir,endSpeed,brakeTime);
}
// sudah
void sline(uint8_t sensor, uint8_t speed, int16_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep);

  if(debug != no_debug){
    char d[getStringFormat(FuncSLine).length() + 15] = {0};
    sprintf(d,getStringFormat(FuncSLine).c_str(),nFunc,sensor,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  speed = constrain(speed, 0, 20);
  backBrakeTime = constrain(backBrakeTime,-255,255);

  bool lineDetected = false;
  while(!lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    uint16_t line = 0;
    senData2Bin(&line);

    if((line & sensor) == sensor) lineDetected = true;

    controllerRun(line, speed);
  }

  while(true){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    
    uint16_t line = 0;
    senData2Bin(&line);

    if((line & sensor) == 0x00) break;
    controllerRun(line, speed);
  }
  
  motor_f(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  backBrakeTimeF(speed,backBrakeTime);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void lostline(uint16_t lostLineTime, uint8_t speed, uint16_t runTime, int16_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep);

  if(debug != no_debug){
    char d[getStringFormat(FuncLostLine).length() + 30] = {0};
    sprintf(d,getStringFormat(FuncLostLine).c_str(),nFunc,lostLineTime,speed,runTime,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  lostLineTime = constrain(lostLineTime, 0, 65000);
  speed = constrain(speed, 0, 20);
  runTime = constrain(runTime, 0, 65000);
  backBrakeTime = constrain(backBrakeTime, -30000, 30000);
  bool flag = false;
  unsigned long timeDetect = millis();
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart <= runTime){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint16_t line = 0;
    senData2Bin(&line);
    if(line != 00) flag = false; // jika sensor mendeteksi garis dimanapun itu, gagalkan pendeteksian
    if(line == 0 && flag == false){ // jika sensor tidak mendeteksi garis dan flagnya masih gagal, maka tandai dan mulai hitungn
      flag = true;
      timeDetect = millis();
    }
    if(flag == true && (unsigned long) millis()-timeDetect > lostLineTime) break; // jika sudah ditandai dan waktunya sudah melebihi batas maka break;
    controllerRun(line,speed,false);
  }
  motor_f(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1);
  }
  backBrakeTimeF(speed,backBrakeTime);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0);
  }
}
/*************************************** akhir dari fungsi yang bisa dijalankan pengguna *******************************************/

// ******************************************** MAIN FUNCTION **********************************************************************
void setup(){
  bsensor.begin(115200);
  fsensor.begin(115200);
  pwmtimer.setPeriod(50);
  srv.begin();
  srv.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  pinMode(pwm1, PWM);
  pinMode(pwm2, PWM);
  pinMode(pwm3, PWM);
  pinMode(pwm4, PWM);
  pinMode(buzz, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(btnMode, INPUT_PULLUP);
  pinMode(btnStep, INPUT_PULLUP);
  pinMode(btnDebug, INPUT_PULLUP);
  pinMode(btnGo, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(buzz, 0);
  
  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA0), pulseCountL, RISING);
  attachInterrupt(digitalPinToInterrupt(PA1), pulseCountR, RISING);
  pwmtimer2.setPeriod(50);

  // mpu6050.begin();
  // mpu6050.calcGyroOffsets(true);
  // mpu6050.calcGyroOffsets(false);
  // mpu6050.calcGyroOffsets(true);
  // digitalWrite(buzz, 1);
  // delay(500);
  // digitalWrite(buzz, 0);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.display();
  delay(10);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  // if (!SD.begin(PA4)) {
  //   display.setCursor(0,0);
  //   display.println(getText(25));
  //   display.display();
  //   while(1){
  //     buzzerled(1,1,2,500,100);
  //   }
  // }
  initMazeRobot();
  getLastMode();
  unsigned long tLcd = millis();
  countSetpoint();
  // standby menu 1
  while(true){ 
    if(isBtnGo() && (unsigned long) millis()-tButton >= DELAY_BUTTON){ // jika button go dipencet
      digitalWrite(buzz, 1);
      tButton = millis();
      delay(DELAY_BUZZER);
      digitalWrite(buzz, 0);
      break; // jika mencet go cepet, maka next
    }
    if(isBtnStep() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
      digitalWrite(buzz, 1);
      tButton = millis();
      if(++nStep > mStep) nStep = 0;
      delay(DELAY_BUZZER);
      digitalWrite(buzz, 0);
    }
    if(isBtnMode() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
      digitalWrite(buzz, 1);
      tButton = millis();
      if(++runMode > 3) runMode = 1;
      countSetpoint();
      delay(DELAY_BUZZER);
      digitalWrite(buzz, 0);
    }
    if(isBtnDebug() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
      digitalWrite(buzz, 1);
      tButton = millis();
      if(++debug > 2) debug = 0;
      delay(DELAY_BUZZER);
      digitalWrite(buzz, 0);
    }
    if((unsigned long) millis()-tLcd >= 50){
      displayMenu();
      tLcd = millis();
    }
  }

  display.clearDisplay();
  display.setCursor(0,0);
  display.print(getStringFormat(MenuMode)); display.println(runMode);
  display.print(getStringFormat(MenuStep)); display.println(nStep);
  display.print(getStringFormat(MenuDebug));
  switch (debug){
    case no_debug: display.println(getStringFormat(InfoNoDebug)); break;
    case by_func: display.println(getStringFormat(InfoByFunc)); break;
    case by_step: display.println(getStringFormat(InfoByStep)); break;
    default: debug = 0; break;
  }
  display.println();
  display.println();
  display.println(getStringFormat(MenuNormal));
  display.display();
  while(1){ if(isBtnGo() && (unsigned long) millis()-tButton >= DELAY_BUTTON) break; }

  logPrint(F("run"));
  
  tButton = millis();
  nFunc = 0;
  mStep = -1; // mulai dari -1 sebelum ke setpoint 0
  setLastMode();

  startTime = millis(); // catat waktu pertama mulai
  switch (runMode){
    case 1: mode1(); break;
    case 2: mode2(); break;
    case 3: mode3(); break;
  }

  char tf[15] = {0};
  dtostrf((millis()-startTime)/1000.0,10,3,tf);
  char d[getStringFormat(InfoYourTime).length() + 10] = {0};
  sprintf(d,getStringFormat(InfoYourTime).c_str(),tf);
  logPrint(d);
}

void loop(){

}