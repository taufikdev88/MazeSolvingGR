#include "MazeSolvingGR.h"

/*
 * Pin declaration
 */
#ifndef ROBOT_NEW
  #define pwm1 PB8
  #define pwm2 PB9
  #define pwm3 PB6
  #define pwm4 PB7
#else
  #define pwm1 PB0
  #define pwm2 PB1
  #define pwm3 PB8
  #define pwm4 PB9
#endif

#define buzz PA8
#define btnMode PA15
#define btnStep PB3
#define btnDebug PB5
#define btnGo PB4
/*
 * Object declaration
 */
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
HardwareTimer pwmtimer(4);
#ifdef ROBOT_NEW
HardwareTimer pwmtimer2(3);
#endif
/*
 * Global variable
 */
#define DELAY_BUZZER 50
#define DELAY_BUTTON 200
// sensor 
#define fsensor Serial2
#define bsensor Serial1
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
bool senData[8] = {0}; // menyimpan nilai sensor
char d[50] = {0}; // variable untuk digunakan bersama untuk sprint log
String mazeLog[2]; // variable untuk log terakhir saat menjalankan mode debugging

// **************************************************************************************************

/************************************* fungsi yang tidak di tampilkan ke pengguna *******************/
#ifdef ROBOT_NEW
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
  detachInterrupt(digitalPinToInterrupt(PA1));
  int rpm = 60*1000 / (millis()-lastTimeL)*counterL;
  lastTimeL = millis();
  counterL = 0;
  attachInterrupt(digitalPinToInterrupt(PA1),pulseCountL,RISING);
  return rpm;
}
int getRPMR(){
  detachInterrupt(digitalPinToInterrupt(PA8));
  int rpm = 60*1000 / (millis()-lastTimeR)*counterR;
  lastTimeR = millis();
  counterR = 0;
  attachInterrupt(digitalPinToInterrupt(PA8),pulseCountR,RISING);
  return rpm;
}
#endif

// ----------------------- fungsi independen / tidak bergantung ke fungsi yang lain -- posisi atas
// fungsi pembacaan nilai dari sensor depan dan belakang termasuk dengan parsing dan menyimpan ke variable senData
void readSensor(bool wichSensor){
  HardwareSerial* os;
  bool temp[8] = { 0 };
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  os->write('A');
  bool isSuccess = false;
  unsigned long timewait = millis();
  for(uint8_t i=0; i<10; i++){
    while(!os->available()){
      if((unsigned long) millis()-timewait > 10) break;
    };

    char in = os->read();
    if(i == 0){
      if( in != 'H') break;
    } else if ( i>0 && i<9){
      if(!isDigit(in)) break;
      temp[i-1] = (in == '0' ? 0 : 1);
    } else if(i == 9){
      if( in != 'T' ) break;
      for( i =0; i<8; i++){
        senData[i] = temp[i];
      }
      i = 10; // keluar loop
      isSuccess = true;
    }
  }
  // jika pembacaan timeout, set senData ke 0 semua
  if(!isSuccess){
    for(uint8_t i=0; i<8; i++){
      senData[i] = 0;
    }
  }
}
// fungsi pembacaan tombol  
#define MAX_DETECT 7
#define MIN_DETECT 4
#define isBtn1 isBtnMode
#define isBtn2 isBtnStep
#define isBtn3 isBtnDebug
#define isBtn4 isBtnGo
bool isBtnGo(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btnGo) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
bool isBtnStep(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btnStep) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
bool isBtnMode(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btnMode) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
bool isBtnDebug(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btnDebug) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
// fungsi untuk mengambil format string dari sd card untuk menghemat flash memory untuk keperluan log
String getText(uint8_t i){
  char filename[16] = {0};
  sprintf(filename,"program/%d.txt",i);
  File file;
  file = SD.open(filename);
  String temp = "";
  if(file){
    while(file.available()){
      temp += (char) file.read();
    }
    file.close();
  }
  return temp;
}
const char mode_n[3] = "md";
// fungsi mengambil last mode yang dijalankan oleh robot
void getLastMode(){
  File file;
  if(!SD.exists(mode_n)){
    runMode = 1;
    file = SD.open(mode_n, FILE_WRITE);
    file.write('1');
  } else {
    file = SD.open(mode_n);
    switch ((char) file.read()){
      case '1': runMode = 1; break;
      case '2': runMode = 2; break;
      case '3': runMode = 3; break;
    }
  }
  file.close();
}
// fungsi menyimpan last mode yang dijalankan oleh robot
void setLastMode(){
  SD.remove(mode_n);
  File file;
  switch(runMode){
    case 1: file = SD.open(mode_n, FILE_WRITE); file.write('1'); break;
    case 2: file = SD.open(mode_n, FILE_WRITE); file.write('2'); break;
    case 3: file = SD.open(mode_n, FILE_WRITE); file.write('3'); break;
  }
  file.close();
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
    case no_debug: display.println(getText(13)); break;
    case by_func: display.println(getText(14)); break;
    case by_step: display.println(getText(15)); break;
  }
  display.println(getText(33));
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
void senData2Bin(uint8_t *line){
  if(senData[0]) *line = *line | 0b10000000;
  if(senData[1]) *line = *line | 0b01000000;
  if(senData[2]) *line = *line | 0b00100000;
  if(senData[3]) *line = *line | 0b00010000;
  if(senData[4]) *line = *line | 0b00001000;
  if(senData[5]) *line = *line | 0b00000100;
  if(senData[6]) *line = *line | 0b00000010;
  if(senData[7]) *line = *line | 0b00000001;
}
// fungsi untuk menjalankan motor kanan kiri berdasarkan skala pwm speed 0-20 
#define PWM_RESOLUTION 3600
void kinematik(int8_t leftSpeed, int8_t rightSpeed){
  float fSpeed = 0.0;
  if(leftSpeed < 0){
    fSpeed = (float) leftSpeed / -20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + leftMotorStartPwmB), 0, PWM_RESOLUTION);
    pwmWrite(pwm1, fSpeed);
    pwmWrite(pwm2, 0);
  } else {
    fSpeed = (float) leftSpeed / 20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + leftMotorStartPwmF), 0, PWM_RESOLUTION);
    pwmWrite(pwm1, 0);
    pwmWrite(pwm2, fSpeed);
  }
  if(rightSpeed < 0){
    fSpeed = (float) rightSpeed / -20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + rightMotorStartPwmB), 0, PWM_RESOLUTION);
    pwmWrite(pwm3, fSpeed);
    pwmWrite(pwm4, 0);
  } else {
    fSpeed = (float) rightSpeed / 20.0;
    fSpeed = constrain(((fSpeed * PWM_RESOLUTION) + rightMotorStartPwmF), 0, PWM_RESOLUTION);
    pwmWrite(pwm3, 0);
    pwmWrite(pwm4, fSpeed);
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
  display.print(getText(10)); display.println(runMode);
  display.print(getText(11)); display.println(nStep);
  display.print(getText(12)); 
  switch(debug){
    case 0: display.println(getText(13)); break;
    case 1: display.println(getText(14)); break;
    case 2: display.println(getText(15)); break;
    default: debug = 0; break;
  }
  display.println(getText(16));
  sprintf(d,"rpm_l:%d r:%d", counterL, counterR);
  display.println(d);

  display.setCursor(0,48);
  readSensor(ff);
  sprintf(d,getText(73).c_str(), senData[0], senData[1], senData[2], senData[3], senData[4], senData[5], senData[6], senData[7]);
  display.println(d);
  readSensor(bb);
  sprintf(d,getText(74).c_str(), senData[7], senData[6], senData[5], senData[4], senData[3], senData[2], senData[1], senData[0]);
  display.println(d);
  display.display();
}
// fungsi yang dijalankan saat robot menemukan error
void errorRaised(){
  logPrint(getText(9));
  while(1){    
    motor_f(0,0,0);
    digitalWrite(buzz, 1);
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    uint8_t line = 0;
    senData2Bin(&line);

    delay(DELAY_BUZZER);
    digitalWrite(buzz, 0);
    delay(DELAY_BUZZER);
    if(line != 0) break;
  }
  logPrint(F("run"));
}
// fungsi yang menjalankan motor sesuai PID untuk sensor garis
void controllerRun(uint8_t line, int8_t speed, bool useError = true){
  int8_t error = 0;
  int8_t pwm = 0;
  
  switch(line){
    // garis 2 sensor
    case 0b10000000: error = -21; isErrorDetect = false; break;
    case 0b11000000: error = -19; isErrorDetect = false; break;
    case 0b01000000: error = -16; isErrorDetect = false; break;
    case 0b01100000: error = -12; isErrorDetect = false; break;
    case 0b00100000: error = -10; isErrorDetect = false; break;
    case 0b00110000: error = -5; isErrorDetect = false; break;
    case 0b00010000: error = -2; isErrorDetect = false; break;
    case 0b00001000: error = 2; isErrorDetect = false; break;
    case 0b00001100: error = 5; isErrorDetect = false; break;
    case 0b00000100: error = 10; isErrorDetect = false; break;
    case 0b00000110: error = 12; isErrorDetect = false; break;
    case 0b00000010: error = 16; isErrorDetect = false; break;
    case 0b00000011: error = 19; isErrorDetect = false; break;
    case 0b00000001: error = 21; isErrorDetect = false; break;
    
    // garis 3 sensor
    case 0b11100000: error = -14; isErrorDetect = false; break;
    case 0b01110000: error = -7; isErrorDetect = false; break;
    case 0b00111000: error = -1; isErrorDetect = false; break;
    case 0b00011100: error = 1; isErrorDetect = false; break;
    case 0b00001110: error = 7; isErrorDetect = false; break;
    case 0b00000111: error = 14; isErrorDetect = false; break;

    // garis 4 sensor 
    case 0b11110000: error = -28; isErrorDetect = false; break;
    case 0b00001111: error = 28; isErrorDetect = false; break;

    // normal 
    case 0b00011000: error = 0; isErrorDetect = false; break;
    
    // error di sensor lain tidak begitu parah
    case 0b00011010:
    case 0b00011001:
    case 0b00011011:
    case 0b01011000:
    case 0b10011000:
    case 0b11011000:
    // error di sensor lain tp parah
    case 0b01111000:
    case 0b00011110:
    // error tp imbang di kanan kiri sensor
    case 0b00111100:
    case 0b01111110:
    case 0b10011001:
    case 0b01011010:
    case 0b11011011:
    // invers error tp imbang di kanan kiri sensor
    case 0b11100111:
    case 0b11000011:
    case 0b10000001:
    case 0b01100110:
    case 0b00100100:
    case 0b01000010: error = 0; isErrorDetect = false; break;
    
    // error sensor kondisi tengah
    case 0b00010010:
    case 0b00010001:
    case 0b00010011: error = -1; isErrorDetect = false; break;
    case 0b01001000:
    case 0b10001000:
    case 0b11001000: error = 1; isErrorDetect = false; break;

    // error sensor kondisi samping medium
    case 0b00110010:
    case 0b00110001:
    case 0b00110011: error = -3; isErrorDetect = false; break;
    case 0b01001100:
    case 0b10001100:
    case 0b11001100: error = 3; isErrorDetect = false; break;

    // error sensor kondisi samping high
    // case 0b00100010:
    // case 0b00100001:
    // case 0b00100011: error = -5; isErrorDetect = false; break;
    // case 0b01000100:
    // case 0b10000100:
    // case 0b11000100: error = 5; isErrorDetect = false; break;

    // garis di semua line
    case 0b11111111: error = 0; isErrorDetect = false; break;

    case 0b00000000: 
    error = 0;
    if(!isErrorDetect){
      logPrint(getText(17));
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

      uint8_t line = 0x00;
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
      if(senData[7]){
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
      if(senData[6] && senData[7]){
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
      if(senData[5]) ++side;
      if(senData[6]) ++side;
      if(senData[7]) ++side;
      if(side >= 5) crossDetected = true;
      break;
      case pp3:
      side = 0;
      if(senData[0]) ++side;
      if(senData[1]) ++side;
      if(senData[2]) ++side;
      if(senData[5]) ++side;
      if(senData[6]) ++side;
      if(senData[7]) ++side;
      if(side >= 6) crossDetected = true;
      break;
      case tt1:
      if((dir == fl || dir == ll || dir == sl) && senData[0]){
        crossDetected = true;
      } else 
      if((dir == fr || dir == rr || dir == sr) && senData[7]){
        crossDetected = true;
      }
      break;
      case tt2:
      if((dir == fl || dir == ll || dir == sl) && senData[0] && senData[1]){
        crossDetected = true;
      } else 
      if((dir == fr || dir == rr || dir == sr) && senData[6] && senData[7]){
        crossDetected = true;
      }
      break;
      case tt3:
      if((dir == fl || dir == ll || dir == sl) && senData[0] && senData[1] && senData[2]){
        crossDetected = true;
      } else 
      if((dir == fr || dir == rr || dir == sr) && senData[5] && senData[6] && senData[7]){
        crossDetected = true;
      }
      break;
    }
    if(crossDetected) continue;
    
    uint8_t line = 0;
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
    while(senData[0] || senData[7]){
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
    while(!crossDetected || (!senData[3] && !senData[4])){
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
    while(!crossDetected || (!senData[3] && !senData[4])){
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

    uint8_t line = 0x00;
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
    sprintf(d, getText(72).c_str(),mStep);
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
    sprintf(d, getText(34).c_str(),nFunc,useBuzzer,useLED,numberOfTimes,interval,customOffTime);
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
    sprintf(d,getText(35).c_str(),nFunc);
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
    sprintf(d,getText(36).c_str(),pc);
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
    sprintf(d,getText(37).c_str(),nFunc,(color == black ? getText(38).c_str():getText(39).c_str()));
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
    sprintf(d,getText(40).c_str(),nFunc,(dir==ff?getText(41).c_str():getText(42).c_str()));
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
    sprintf(d,getText(43).c_str(),nFunc,useError,time);
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
    sprintf(d,getText(44).c_str(),nFunc,p1,p2);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}
}
// sudah
void ffspeed(uint8_t l, uint8_t r){
  if(isModeCount) return;
  ++nFunc;
  
  if(debug != no_debug){
    sprintf(d,getText(45).c_str(),nFunc,l,r);
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
    sprintf(d,getText(46).c_str(),nFunc,l,r);
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
    sprintf(d,getText(48).c_str(),nFunc,leftSpeed,rightSpeed,runTime);
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
    sprintf(d,getText(49).c_str(),nFunc,method,dir,speed,brakeTime);
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
    sprintf(d,getText(50).c_str(),nFunc,method,dir,speed,brakeTime,actionTime);
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
    sprintf(d,getText(51).c_str(),nFunc,method,dir,speed,brakeTime,totalActionTime);
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
    sprintf(d,getText(52).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(53).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(54).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(55).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(56).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(57).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(leftSide, 4, speed, backBrakeTime);
}
// sudah
void right(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    sprintf(d,getText(58).c_str(),nFunc,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  turn(rightSide, 6, speed, backBrakeTime);
}
void right8(uint8_t speed, uint8_t backBrakeTime){
  if(isModeCount) return;
  ++nFunc;
  if(mStep < nStep) return;
  
  if(debug != no_debug){
    sprintf(d,getText(59).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(60).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(61).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(62).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(63).c_str(),nFunc,speed,backBrakeTime);
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
    sprintf(d,getText(64).c_str(),nFunc,leftMotorSpeed,rightMotorSpeed,sensor,backBrakeTime);
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
    uint8_t line = 0x00;
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
    uint8_t line = 0x00;
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
    sprintf(d,getText(65).c_str(),leftMotorSpeed,rightMotorSpeed,sensor,backBrakeTime);
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
    uint8_t line = 0x00;
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
    sprintf(d,getText(66).c_str(),nFunc,speed,runTime,backBrakeTime);
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
    sprintf(d,getText(67).c_str(),nFunc,leftSpeed,rightSpeed,timeToDisregardLine);
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
    for(int8_t i=0; i<8; i++){
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
    sprintf(d,getText(68).c_str(),nFunc,runTime,startSpeed,method,dir,endSpeed,brakeTime);
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
    sprintf(d,getText(69).c_str(),nFunc,runTime,startSpeed,method,dir,endSpeed,brakeTime);
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
    sprintf(d,getText(70).c_str(),nFunc,sensor,speed,backBrakeTime);
    logPrint(d);
  }
  if(debug == by_func) {while(digitalRead(btnGo) || (unsigned long) millis()-tButton <= DELAY_BUTTON){}; tButton = millis(); digitalWrite(buzz,1); delay(DELAY_BUZZER); digitalWrite(buzz,0);}

  speed = constrain(speed, 0, 20);
  backBrakeTime = constrain(backBrakeTime,-255,255);

  bool lineDetected = false;
  while(!lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    uint8_t line = 0;
    senData2Bin(&line);

    if((line & sensor) == sensor) lineDetected = true;

    controllerRun(line, speed);
  }

  while(true){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    
    uint8_t line = 0;
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
    sprintf(d,getText(71).c_str(),nFunc,lostLineTime,speed,runTime,backBrakeTime);
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
    uint8_t line = 0;
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
  
  #ifdef ROBOT_NEW
  pinMode(PA1, INPUT);
  pinMode(PA8, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA1), pulseCountL, RISING);
  attachInterrupt(digitalPinToInterrupt(PA8), pulseCountR, RISING);
  pwmtimer2.setPeriod(50);
  #endif

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(10);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  if (!SD.begin(PA4)) {
    display.setCursor(0,0);
    display.println(getText(25));
    display.display();
    while(1){
      buzzerled(1,1,2,500,100);
    }
  }
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
  display.print(getText(26)); display.println(runMode);
  display.print(getText(27)); display.println(nStep);
  display.print(getText(28));
  switch (debug){
    case no_debug: display.println(getText(13)); break;
    case by_func: display.println(getText(14)); break;
    case by_step: display.println(getText(15)); break;
    default: debug = 0; break;
  }
  display.println();
  display.println();
  display.println(getText(30));
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
  sprintf(d,getText(77).c_str(),tf);
  logPrint(d);
}

void loop(){
}