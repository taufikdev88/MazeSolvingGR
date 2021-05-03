#include "MazeCalibration.h"

#ifdef MAZECALIBRATION_H_

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#define pwm1 PB8
#define pwm2 PB9
#define pwm3 PB6
#define pwm4 PB7
#define buzz PA8
#define btnMode PA15
#define btnStep PB3
#define btnDebug PB5
#define btnGo PB4

#define DELAY_BUZZER 50
#define DELAY_BUTTON 200

#define ff 0
#define bb 1

#define fsensor Serial2
#define bsensor Serial1

bool senData[8] = {0}; // menyimpan nilai sensor
char d[50] = {0}; 
unsigned long tLcd = millis();
unsigned long tButton = millis();

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
void printDisplayHome(String text){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(text);
  display.display();
}
void buzzerled(bool useBuzzer, bool useLED, uint8_t numberOfTimes, uint16_t interval, uint16_t customOffTime){  
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
void displayConfig1(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(getText(1));
  display.println(getText(2));
  display.println(getText(3));
  display.println(getText(4));
  display.display();
}
void displayConfig2(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(getText(5));
  display.println(getText(6));
  display.println(getText(7));
  display.println(getText(8));
  display.display();
}
void testSensor(bool wichSensor){
  HardwareSerial* os;
  
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;

  unsigned long timeStart = millis();
  os->write('T');
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome(getText(18));
      delay(3000);
      return;
    }
  }
  
  while(digitalRead(btnGo)){
    readSensor(wichSensor);

    sprintf(d,getText(75).c_str(),senData[0],senData[1],senData[2],senData[3],senData[4],senData[5],senData[6],senData[7]);
    printDisplayHome(d);

    delay(10);
  }
}
void saveSensor(bool wichSensor){
  HardwareSerial* os;
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  unsigned long timeStart = millis();
  os->write('T');
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome(getText(18));
      delay(3000);
      return;
    }
  }
  char C = os->read();
  if(!isDigit(C)){
    printDisplayHome(getText(19));
    delay(3000);
    return;
  }
  os->write('S');
  timeStart = millis();
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome(getText(18));
      delay(3000);
      return;
    }
  }
  C = os->read();
  if(C == 'D'){
    printDisplayHome(getText(20));
  } else {
    printDisplayHome(getText(21));
  }
  delay(3000);  
}
void calibSensor(bool wichSensor){
  HardwareSerial* os;
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  unsigned long timeStart = millis();
  os->write('T');
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome(getText(18));
      delay(3000);
      return;
    }
  }
  char C = os->read();
  if(!isDigit(C)) {
    printDisplayHome(getText(19));
    delay(3000);
    return;
  }
  os->write('C');
  timeStart = millis();
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 12000){
      printDisplayHome(getText(22));
      delay(3000);
      return;
    }

    int timeSpent = ((unsigned long) millis()-timeStart) / 1000;
    sprintf(d,getText(76).c_str(), timeSpent);
    printDisplayHome(d);

    delay(900);
  }
  if(os->read() == 'D'){
    printDisplayHome(getText(23));
  } else {
    printDisplayHome(getText(24));
  }
  delay(3000);
}
void setup(){
  bsensor.begin(115200);
  fsensor.begin(115200);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(btnMode, INPUT_PULLUP);
  pinMode(btnStep, INPUT_PULLUP);
  pinMode(btnDebug, INPUT_PULLUP);
  pinMode(btnGo, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(buzz, 0);
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
}
void loop(){
  uint8_t menu1 = 0;
  bool stepMenu = 0;

  switch (stepMenu){
    case 0:
      if(isBtn1() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
        digitalWrite(buzz, 1);
        tButton = millis();
        menu1 = 1;
        stepMenu = 1;
        delay(DELAY_BUZZER);
      }
      if(isBtn2() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
        digitalWrite(buzz, 1);
        menu1 = 2;
        stepMenu = 1;
        tButton = millis();
        delay(DELAY_BUZZER);
      }
      if(isBtn3() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
        digitalWrite(buzz, 1);
        menu1 = 3;
        stepMenu = 1;
        tButton = millis();
        delay(DELAY_BUZZER);
      }
    case 1:
      if(isBtn1() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
        digitalWrite(buzz, 1);
        tButton = millis();
        delay(DELAY_BUZZER);
        digitalWrite(buzz, 0);
        switch (menu1)
        {
        case 1:
          calibSensor(ff);
          break;
        case 2:
          testSensor(ff);
          break;
        case 3:
          saveSensor(ff);
          break;
        default:
          break;
        }
      }
      if(isBtn2() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
        digitalWrite(buzz, 1);
        tButton = millis();
        delay(DELAY_BUZZER);
        digitalWrite(buzz, 0);
        switch (menu1)
        {
        case 1:
          calibSensor(bb);
          break;
        case 2:
          testSensor(bb);
          break;
        case 3:
          saveSensor(bb);
          break;
        default:
          break;
        }
      }

      if(isBtn4() && (unsigned long) millis()-tButton >= DELAY_BUTTON){
        digitalWrite(buzz, 1);
        tButton = millis();
        stepMenu = 0;
        delay(DELAY_BUZZER);
      }
      break;
    default: stepMenu = 0; break;
  }

  digitalWrite(buzz,0);
  if((unsigned long) millis()-tLcd >= DELAY_BUTTON){
    tLcd = millis();
    if(stepMenu == 0) displayConfig1(); else displayConfig2();
  }
}

#endif

''''[]