#include "MazeSolvingGR.h"

Adafruit_SSD1306 display(OLED_RESET);
/*
 * Global variable
 */
bool isUseBuzzer = true;
bool isTraceForward = true;
bool isUseError = true;
bool isErrorDetect = false;
bool isMirror = false;
uint16_t errorTime = 500;
uint16_t pcTime = 100;
#define defaultKp 2
#define defaultKi 0.01
#define defaultKd 0.5
float Kp = defaultKp;
float Ki = defaultKi;
float Kd = defaultKd;
float iError = 0;
float dError = 0;
uint8_t leftMotorStartPwmF = 0;
uint8_t rightMotorStartPwmF = 0;
uint8_t leftMotorStartPwmB = 0;
uint8_t rightMotorStartPwmB = 0;
uint8_t linecount = 2;
bool senData[8] = {0};
unsigned long startTime = millis();
unsigned long errorStart = millis();

// sudah
void buzzerled(bool isUseBuzzer, bool isUseLed, uint8_t numberOfTimes, uint16_t interval, uint16_t customOffTime = 0){
  bool state = true;
  interval = constrain(interval, 0, 65000);
  customOffTime = constrain(customOffTime, 0, 65000);
  for(uint16_t i=0; i<(uint16_t) numberOfTimes*2; i++){
    if(state){
      if(isUseBuzzer) digitalWrite(buzz, 1);
      if(isUseLed) digitalWrite(LED_BUILTIN, 1);
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
void logPrint(String newLog){
  newLog = newLog + " " + String((unsigned long) millis()-startTime); 
  display.clearDisplay();
}
// sudah
void start(bool useBuzzer, uint16_t pc){
  pc = constrain(pc, 0, 65000);
  isUseBuzzer = useBuzzer;
  pcTime = pc;
}
// sudah
void resetTimer(){
  startTime = millis();
}
// sudah 
void pctimer(uint16_t pc){
  pc = constrain(pc, 0, 65000);
  pcTime = pc;
}
// sudah
void linecolor(bool color){
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
  if(dir == ff){
    isTraceForward = true;
  } else {
    isTraceForward = false;
  }
}
// sudah
void error(bool useError, uint16_t time){
  isUseError = useError;
  errorTime = time;
}
// sudah
void controller(float p1, float p2, float p3){
  Kp = p1;
  Ki = p2;
  Kd = p3;
  Kp = constrain(Kp, -1.0, 10.0);
  Ki = constrain(Ki, -1.0, 10.0);
  Kd = constrain(Kd, -1.0, 10.0);
  if(Kp == -1.0) Kp = defaultKp;
  if(Ki == -1.0) Ki = defaultKi;
  if(Kd == -1.0) Kd = defaultKd;
}
// sudah
void ffspeed(uint8_t l, uint8_t r){
  leftMotorStartPwmF = l;
  rightMotorStartPwmF = r;
}
// sudah
void bbspeed(uint8_t l, uint8_t r){
  leftMotorStartPwmB = l;
  rightMotorStartPwmB = r;
}
// sudah
void linetrack(uint8_t l){
  l = constrain(l,2,4);
  linecount = l;
}
// sudah
void motor(int8_t leftSpeed, int8_t rightSpeed, uint16_t runTime = 0){
  leftSpeed = constrain(leftSpeed, -20, 20);
  rightSpeed = constrain(rightSpeed, -20, 20);
  float fSpeed = 0.0;
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart <= runTime){
    if(leftSpeed < 0){
      // mundur
      fSpeed = (float) leftSpeed / -20.0;
      fSpeed = constrain(((fSpeed * 255.0) + leftMotorStartPwmB), 0, 255);
      analogWrite(pwm1, fSpeed );
      analogWrite(pwm2, 0);
    } else {
      // maju
      fSpeed = (float) leftSpeed / 20.0;
      fSpeed = constrain(((fSpeed * 255.0) + leftMotorStartPwmF), 0, 255);
      analogWrite(pwm1, 0);
      analogWrite(pwm2, fSpeed);
    }
    if(rightSpeed < 0){
      // mundur
      fSpeed = (float) rightSpeed / -20.0;
      fSpeed = constrain(((fSpeed * 255.0) + rightMotorStartPwmB), 0, 255);  
      analogWrite(pwm3, fSpeed);
      analogWrite(pwm4, 0);
    } else {
      // maju
      fSpeed = (float) rightSpeed / 20.0;
      fSpeed = constrain(((fSpeed * 255.0) + rightMotorStartPwmF), 0, 255);
      analogWrite(pwm3, 0);
      analogWrite(pwm4, fSpeed);
    }
  }
  if(runTime != 0){
    analogWrite(pwm1, 0);
    analogWrite(pwm2, 0);
    analogWrite(pwm3, 0);
    analogWrite(pwm4, 0);
  }
}
// sudah
void readSensor(bool wichSensor, bool debug = false){
  HardwareSerial* os;
  bool temp[8] = { 0 };
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  os->write('A');
  for(uint8_t i=0; i<10; i++){
    while(!os->available());
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
    }
  }
}
// sudah
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
// sudah
void errorRaised(){
  motor(0,0,0);
  logPrint("line missing");
  while(1){    
    buzzerled(1,1,30,200);
  }
}
// sudah
void controllerRun(uint8_t line, int8_t speed, bool useError = true){
  int8_t error = 0;
  int8_t pwm = 0;
  
  switch(line){
    // garis 2 sensor
//    case 0b10000000: error = -14; isErrorDetect = false; break;
//    case 0b11000000: error = -12; isErrorDetect = false; break;
    case 0b01000000: error = -10; isErrorDetect = false; break;
    case 0b01100000: error = -8; isErrorDetect = false; break;
    case 0b00100000: error = -6; isErrorDetect = false; break;
    case 0b00110000: error = -4; isErrorDetect = false; break;
    
    case 0b00010000: error = -2; isErrorDetect = false; break;
    case 0b00001000: error = 2; isErrorDetect = false; break;
    case 0b00011000: error = 0; isErrorDetect = false; break;

    case 0b00001100: error = 4; isErrorDetect = false; break;
    case 0b00000100: error = 6; isErrorDetect = false; break;
    case 0b00000110: error = 8; isErrorDetect = false; break;
    case 0b00000010: error = 10; isErrorDetect = false; break;
//    case 0b00000011: error = 12; isErrorDetect = false; break;
//    case 0b00000001: error = 14; isErrorDetect = false; break;

    // garis 3 sensor
//    case 0b11100000: if(linecount >= 3) error = -8; isErrorDetect = false; break;
    case 0b01110000: if(linecount >= 3) error = -6; isErrorDetect = false; break;
    case 0b00111000: if(linecount >= 3) error = -2; isErrorDetect = false; break;
    case 0b00011100: if(linecount >= 3) error = 2; isErrorDetect = false; break;
    case 0b00001110: if(linecount >= 3) error = 6; isErrorDetect = false; break;
//    case 0b00000111: if(linecount >= 3) error = 8; isErrorDetect = false; break;

    // garis 4 sensor 
//    case 0b11110000: if(linecount == 4) error = -8; isErrorDetect = false; break;
    case 0b01111000: if(linecount == 4) error = -4; isErrorDetect = false; break;
    case 0b00111100: if(linecount == 4) error = 0; isErrorDetect = false; break;
    case 0b00011110: if(linecount == 4) error = 4; isErrorDetect = false; break;
//    case 0b00001111: if(linecount == 4) error = 8; isErrorDetect = false; break;

    // garis di semua line
    case 0b11111111:
    case 0b01111110: error = 0; isErrorDetect = false; break;

    case 0b00000000: 
    error = 0;
    if(!isErrorDetect){
      logPrint("count error");
      errorStart = millis();
      isErrorDetect = true;
    }
    break;
  }

  if(useError && isErrorDetect && (unsigned long) millis()-errorStart >= errorTime && isUseError){
    errorRaised();
  }

  pwm = Kp*error + Ki*iError + Kd*(error-dError);
  dError = error;
  iError += error;
  iError = constrain(iError, -500, 500);

  int8_t s1 = speed+pwm;
  int8_t s2 = speed-pwm;
  
  if(isTraceForward) motor(s1, s2, 0);
  else motor(-s2, -s1, 0);
}
// sudah
void backBrakeTimeF(uint8_t speed, int16_t brakeTime){
  // brakeTime
  if(brakeTime > 0){
    if(isTraceForward){
      motor(-speed,-speed, brakeTime);
    } else {
      motor(speed, speed, brakeTime);
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
  motor(0,0,0);
}
// sudah
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
    motor(0,0,0);
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
        motor(speed,speed,0);
      } else {
        readSensor(bb);
        motor(-speed,-speed,0);
      }
    }
    motor(0,0,0);
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
        motor(-speed,speed,0);
        readSensor(ff);
      } else {
        motor(speed,-speed,0);
        readSensor(bb);
      }
      if(senData[0]) crossDetected = true;
    }
    break;
    case rr:
    while(!crossDetected || (!senData[3] && !senData[2])){
      if(isTraceForward){
        motor(speed,-speed,0);
        readSensor(ff);
      } else {
        motor(-speed,speed,0);
        readSensor(bb);
      }
      if(senData[7]) crossDetected = true;
    }
    break; 
  }
  motor(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void line(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime){
  speed = constrain(speed, -20, 20);
  brakeTime = constrain(brakeTime, -30000, 30000);

  logPrint((String)"line " + method + "," + dir);
  runAndDetect(method, dir, speed, brakeTime, -1);
}
// sudah
void linet(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime){
  speed = constrain(speed, -20, 20);
  brakeTime = constrain(brakeTime, -30000, 30000);
  
  logPrint((String)"linet " + method + "," + dir);
  runAndDetect(method, dir, speed, brakeTime, actionTime);
}
// sudah
void timeline(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t totalActionTime){
  speed = constrain(speed, -20, 20);
  brakeTime = constrain(brakeTime, -30000, 30000);

  logPrint((String)"timeline " + method + "," + dir);
  while((unsigned long) millis()-startTime < totalActionTime);
  runAndDetect(method, dir, speed, brakeTime, -1);
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
      motor(-speed, speed, 0);
      break;
      case rightSide:
      motor(speed, -speed, 0);
      break;
    } 
  }
  // stop
  motor(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  // reverse turning
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart<backBrakeTime){
    switch(dir){
      case leftSide:
      motor(speed, -speed, 0);
      break;
      case rightSide:
      motor(-speed, speed, 0);
      break;
    }
  }
  // stop
  motor(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void left(uint8_t speed, uint8_t backBrakeTime){
  logPrint("left");
  turn(leftSide, 1, speed, backBrakeTime);
}
void left1(uint8_t speed, uint8_t backBrakeTime){
  logPrint("left1");
  turn(leftSide, 0, speed, backBrakeTime);
}
void left2(uint8_t speed, uint8_t backBrakeTime){
  logPrint("left2");
  turn(leftSide, 1, speed, backBrakeTime);
}
void left3(uint8_t speed, uint8_t backBrakeTime){
  logPrint("left3");
  turn(leftSide, 2, speed, backBrakeTime);
}
void left4(uint8_t speed, uint8_t backBrakeTime){
  logPrint("left4");
  turn(leftSide, 3, speed, backBrakeTime);
}
void left5(uint8_t speed, uint8_t backBrakeTime){
  logPrint("left5");
  turn(leftSide, 4, speed, backBrakeTime);
}
// sudah
void right(uint8_t speed, uint8_t backBrakeTime){
  logPrint("right");
  turn(rightSide, 6, speed, backBrakeTime);
}
void right8(uint8_t speed, uint8_t backBrakeTime){
  logPrint("right8");
  turn(rightSide, 7, speed, backBrakeTime);
}
void right7(uint8_t speed, uint8_t backBrakeTime){
  logPrint("right7");
  turn(rightSide, 6, speed, backBrakeTime);
}
void right6(uint8_t speed, uint8_t backBrakeTime){
  logPrint("right6");
  turn(rightSide, 5, speed, backBrakeTime);
}
void right5(uint8_t speed, uint8_t backBrakeTime){
  logPrint("right5");
  turn(rightSide, 4, speed, backBrakeTime);
}
void right4(uint8_t speed, uint8_t backBrakeTime){
  logPrint("right4");
  turn(rightSide, 3, speed, backBrakeTime);
}
// sudah
void exline(int8_t leftMotorSpeed, int8_t rightMotorSpeed, uint8_t sensor, int16_t backBrakeTime){
  backBrakeTime = constrain(backBrakeTime, -255, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, -20, 20);
  rightMotorSpeed = constrain(rightMotorSpeed, -20, 20);
  bool lineDetected = false;
  
  logPrint("exline");
  while(!lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint8_t line = 0x00;
    senData2Bin(&line);  
    if((line & sensor) == sensor) lineDetected = true;
    if(isTraceForward){
      motor(leftMotorSpeed, rightMotorSpeed, 0);
    } else {
      motor(-leftMotorSpeed, -rightMotorSpeed, 0);
    }
  }
  while(lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint8_t line = 0x00;
    senData2Bin(&line);  
    if((line & sensor) == 0x00) lineDetected = false;  
    if(isTraceForward){
      motor(leftMotorSpeed, rightMotorSpeed, 0);
    } else {
      motor(-leftMotorSpeed, -rightMotorSpeed, 0);
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
        motor(leftMotorSpeed, rightMotorSpeed, 0);
      } else {
        motor(-leftMotorSpeed, -rightMotorSpeed, 0);
      }
    } else {
      if(backBrakeTime < 0){
        motor(-leftMotorSpeed, -rightMotorSpeed, 0);
      } else {
        motor(leftMotorSpeed, rightMotorSpeed, 0);
      }
    }
  }
  motor(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void exturn(int8_t leftMotorSpeed, int8_t rightMotorSpeed, uint8_t sensor, int16_t backBrakeTime){
  backBrakeTime = constrain(backBrakeTime, -255, 255);
  leftMotorSpeed = constrain(leftMotorSpeed, -20, 20);
  rightMotorSpeed = constrain(rightMotorSpeed, -20, 20);
  bool lineDetected = false;
  
  logPrint("exturn");  
  while(!lineDetected){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);
    uint8_t line = 0x00;
    senData2Bin(&line);
    if((line & sensor) == sensor) lineDetected = true;
    
    if(isTraceForward){
      motor(leftMotorSpeed, rightMotorSpeed, 0);
    } else {
      motor(-leftMotorSpeed, -rightMotorSpeed, 0);
    }
  }
  motor(0,0,0);
  if(isUseBuzzer){
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1); 
  }
  unsigned long timeStart = millis();
  while((unsigned long) millis()-timeStart<abs(backBrakeTime)){
    if(isTraceForward){
      if(backBrakeTime < 0){
        motor(leftMotorSpeed, rightMotorSpeed, 0);
      } else {
        motor(-leftMotorSpeed, -rightMotorSpeed, 0);
      }
    } else {
      if(backBrakeTime < 0){
        motor(-leftMotorSpeed, -rightMotorSpeed, 0);
      } else {
        motor(leftMotorSpeed, rightMotorSpeed, 0);
      }
    }
  }
  motor(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  if(isUseBuzzer){
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0); 
  }
}
// sudah
void linedelay(uint8_t speed, uint16_t runTime, int16_t backBrakeTime){
  speed = constrain(speed, 0, 20);
  runTime = constrain(runTime, 0, 65000);
  backBrakeTime = constrain(backBrakeTime, -255, 255);

  logPrint("linedelay");
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
    if(isTraceForward) motor(speed, speed, backBrakeTime);
    else motor(-speed, -speed, backBrakeTime);
  } else if(backBrakeTime > 0){
    if(isTraceForward) motor(-speed, -speed, backBrakeTime);
    else motor(speed, speed, backBrakeTime);
  }
  motor(0,0,0);
  if(backBrakeTime == 0) delay(DELAY_BUZZER);
  digitalWrite(buzz, 0);
  digitalWrite(LED_BUILTIN, 0);
}
// sudah
void linefind(int8_t leftSpeed, int8_t rightSpeed, uint16_t timeToDisregardLine){   
  leftSpeed = constrain(leftSpeed, -20, 20);
  rightSpeed = constrain(rightSpeed, -20, 20);
  timeToDisregardLine = constrain(timeToDisregardLine, 0, 65000);
  bool isFound = false;

  logPrint("linefind");
  unsigned long timeStart = millis();
  while(!isFound){
    if(isTraceForward) readSensor(ff);
    else readSensor(bb);

    motor(leftSpeed, rightSpeed, 0);
    for(int8_t i=0; i<8; i++){
      if(senData[i]){
        if((unsigned long) millis()-timeStart > timeToDisregardLine){
          isFound = true;
        }
      }
    }
  }
  motor(0,0,0);
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
  linedelay(startSpeed, runTime, 0);
  line(method,dir,endSpeed,brakeTime);
}
// sudah
void linetline(uint16_t runTime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t brakeTime){
  linet(method,dir,startSpeed,0,runTime);
  line(method,dir,endSpeed,brakeTime);
}
// sudah
void sline(uint8_t sensor, uint8_t speed, int16_t backBrakeTime){
  speed = constrain(speed, 0, 20);
  backBrakeTime = constrain(backBrakeTime,-255,255);

  logPrint("sline");
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
  
  motor(0,0,0);
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
  lostLineTime = constrain(lostLineTime, 0, 65000);
  speed = constrain(speed, 0, 20);
  runTime = constrain(runTime, 0, 65000);
  backBrakeTime = constrain(backBrakeTime, -30000, 30000);
  bool flag = false;
  unsigned long timeDetect = millis();
  logPrint("lostline");
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
  motor(0,0,0);
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
#define MAX_DETECT 20
#define MIN_DETECT 15
#define T_BUTTON 250
bool isBtn1(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btn1) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
bool isBtn2(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btn2) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
bool isBtn3(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btn3) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
bool isBtn4(){
  uint8_t cnt = 0;
  for(uint8_t i=0; i<MAX_DETECT; i++){
    if(digitalRead(btn4) == 0) cnt++;
    delayMicroseconds(10);
  }
  if(cnt > MIN_DETECT) return true;
  return false;
}
// sudah
void printDisplayHome(String text){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(text);
  display.display();
}
// sudah
void testSensor(bool wichSensor){
  HardwareSerial* os;
  
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;

  unsigned long timeStart = millis();
  os->write('T');
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome("Sensor tidak terdeteksi");
      delay(3000);
      return;
    }
  }

  printDisplayHome("Deteksi");
  delay(100);
  
  while(digitalRead(btn4)){
    readSensor(wichSensor);

    char D[20] = {0};
    sprintf(D, "%d%d%d%d%d%d%d%d\n%d%d%d%d%d%d%d%d", 
      senData[0], senData[1], senData[2], senData[3], senData[4], senData[5], senData[6], senData[7],
      !senData[0], !senData[1], !senData[2], !senData[3], !senData[4], !senData[5], !senData[6], !senData[7]
    );
    printDisplayHome(D);

    delay(10);
  }
}
// sudah
void saveSensor(bool wichSensor){
  HardwareSerial* os;
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  unsigned long timeStart = millis();
  os->write('T');
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome("Sensor tidak terdeteksi");
      delay(3000);
      return;
    }
  }
  char C = os->read();
  if(isDigit(C)){
    printDisplayHome((String) "Jumlah sensor: " + C);
  } else {
    printDisplayHome((String) "Sensor salah!");
    delay(3000);
    return;
  }
  os->write('S');
  timeStart = millis();
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome("Sensor tidak terdeteksi");
      delay(3000);
      return;
    }
  }
  C = os->read();
  if(C == 'D'){
    printDisplayHome("Berhasil menyimpan");
  } else {
    printDisplayHome("Gagal menyimpan");
  }
  delay(3000);  
}
// sudah
void calibSensor(bool wichSensor){
  HardwareSerial* os;
  if(wichSensor == ff) os = &fsensor;
  else os = &bsensor;
  unsigned long timeStart = millis();
  os->write('T');
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 200){
      printDisplayHome("Sensor tidak terdeteksi");
      delay(3000);
      return;
    }
  }
  char C = os->read();
  if(isDigit(C)){
    printDisplayHome((String) "Jumlah sensor: " + C);
  } else {
    printDisplayHome((String) "Sensor salah!");
    delay(3000);
    return;
  }
  os->write('C');
  timeStart = millis();
  while(!os->available()){
    if((unsigned long) millis()-timeStart >= 12000){
      printDisplayHome("Kalibrasi timeout");
      delay(3000);
      return;
    }
    int timeSpent = ((unsigned long) millis()-timeStart) / 1000;
    printDisplayHome(String(timeSpent));
    delay(900);
  }
  if(os->read() == 'D'){
    printDisplayHome("Kalibrasi selesai");
  } else {
    printDisplayHome("Kalibrasi invalid");
  }
  delay(3000);
}
// sudah
void showIcon(String filename){
  File file;
  filename = "icon/" + filename;
  file = SD.open(filename);
  if(file){
    uint16_t i = 0;
    uint8_t icon[1024] = { 0 };
    filename = "";
    while(file.available()){
      char C = file.read();
      if(isDigit(C)){
        filename += C;
      } else {
        icon[i] = filename.toInt();
        filename = "";
        if(++i == 1024){
          display.clearDisplay();
          display.setCursor(0,0);
          display.drawBitmap(0,0,icon,128,64,WHITE);
          display.display();
        }
      }
    }
    file.close();
  }
}
// sudah
void printMenu(String *menuArray, uint8_t index, uint8_t totalSize){
  display.clearDisplay();
  display.setCursor(0,0);
  for(uint8_t i=0; i<totalSize; i++){
    if(i == index){
      display.setTextColor(BLACK, WHITE);
    } else {
      display.setTextColor(WHITE);
    }
    display.println(*(menuArray+i));
  }
  display.display();
  display.setTextColor(WHITE);
}
// sudah
void showMenu(int8_t *menu0, int8_t *menu1, int8_t *menu2){
  Serial.print(*menu0);
  Serial.print(" -> ");
  Serial.print(*menu1);
  Serial.print(" -> ");
  Serial.println(*menu2);
  
  *menu0 = constrain(*menu0,0,2);
  *menu1 = constrain(*menu1,0,3);
  switch(*menu0){
    case 0:
    switch(*menu1){
      case 0: 
      showIcon("run.txt"); 
      display.setCursor(0,55);
      display.print("      RUN PROGRAM    ");
      display.display();
      break;
      case 1: 
      showIcon("calib.txt");
      display.setCursor(0,55);
      display.print("  SENSOR CALIBRATION ");
      display.display();
      break;
      case 2:
      showIcon("cek.txt");
      display.setCursor(0,55);
      display.print("     SENSOR TEST     ");
      display.display();
      break;
      case 3:
      showIcon("save.txt");
      display.setCursor(0,55);
      display.print("SENSOR SAVE TO EEPROM");
      display.display();
      break;
    }
    break;
    case 1:
    switch(*menu1){
      case 1:
      case 2:
      case 3:
      String menuArray[2] = {
        "Front Sensor",
        "Back Sensor"
      };
      *menu2 = constrain(*menu2,0,1);
      printMenu(menuArray, *menu2, sizeof(menuArray)/sizeof(menuArray[0]));
      break;
    }
    break;
    case 2:
    if(*menu1 == 1 && *menu2 == 0){
      // calib sensor depan
      calibSensor(ff);
    } else
    if(*menu1 == 1 && *menu2 == 1){
      // calib sensor belakang
      calibSensor(bb);
    } else 
    if(*menu1 == 2 && *menu2 == 0){
      // test sensor depan
      testSensor(ff);
    } else
    if(*menu1 == 2 && *menu2 == 1){
      // test sensor belakang
      testSensor(bb);
    } else
    if(*menu1 == 3 && *menu2 == 0){
      // simpan sensor depan
      saveSensor(ff);
    } else 
    if(*menu1 == 3 && *menu2 == 1){
      // simpan sensor belakang
      saveSensor(bb);
    }
    break;
  }
}
//*************************************************** PROHIBITED SECTION *************
/*
 * Main Function
 */
void setup(){
  // setup robot pin declaration and initialization
  Serial.begin(115200);
  bsensor.begin(115200);
  fsensor.begin(115200);

  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(buzz, OUTPUT);
  
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(buzz, 0);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display.display();
  delay(500);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  Serial.print("Initializing SD card on port: ");
  Serial.print(PA4);
  Serial.print("...");
  if (!SD.begin(PA4)) {
    Serial.println("initialization failed!");
    display.println("SD CARD ERROR!!!");
    display.display();
    while(1);
  }
  Serial.println("initialization done.");

  int8_t menu0 = 0;
  int8_t menu1 = 0;
  int8_t menu2 = 0;
  
  unsigned long tButton = millis();
  bool standby = true;
  showMenu(&menu0, &menu1, &menu2);
  
  while(standby){
    if(isBtn1() && (unsigned long) millis()-tButton >= T_BUTTON){
      tButton = millis();
      switch(menu0){
        case 0: --menu1; break;
        case 1: --menu2; break;
      }
      showMenu(&menu0, &menu1, &menu2);
    }
    if(isBtn2() && (unsigned long) millis()-tButton >= T_BUTTON){
      tButton = millis();
      switch(menu0){
        case 0: ++menu1; break;
        case 1: ++menu2; break;
      }
      showMenu(&menu0, &menu1, &menu2);
    }
    if(isBtn3() && (unsigned long) millis()-tButton >= T_BUTTON){
      if(menu1 == 0) break;
      tButton = millis();
      ++menu0;
      showMenu(&menu0, &menu1, &menu2);
    }
    if(isBtn4() && (unsigned long) millis()-tButton >= T_BUTTON){
      tButton = millis();
      --menu0;
      showMenu(&menu0, &menu1, &menu2);
    }
  }

  int8_t mode = 0;
  startTime = millis();  
  logPrint("Hello MazeRobot");
  switch(mode){
    case 0: isMirror = false; mode1(); break;
    case 1: isMirror = true; mode1(); break;
    case 2: isMirror = false; mode2(); break;
    case 3: isMirror = true; mode2(); break;
    case 4: isMirror = false; mode3(); break;
    case 5: isMirror = true; mode3(); break;
  }
  // end
  logPrint("End");
}
/*
 * Loop , for debugging
 */
void loop(){ 
  buzzerled(0,1,1,50);
}