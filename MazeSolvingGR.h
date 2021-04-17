#ifndef MAZESOLVINGGR_H_
#define MAZESOLVINGGR_H_

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#define OLED_RESET 4
/*
 * Global name
 */
#define DELAY_BUZZER 10
// sensor 
#define fsensor Serial2
#define bsensor Serial1
// line color
#define black 0
#define white 1
// run forward / backward
#define ff 0
#define bb 1
// brake mode
#define fast 1
#define slow 0
// line detection mode
#define pp0 0
#define pp1 1
#define pp2 2
#define pp3 3
#define tt1 4
#define tt2 5
#define tt3 6
#define pp pp0
#define tt tt2
// sensor detection side
#define none -1
#define leftSide 0
#define rightSide 1
// after crossing mode
// ff -> robot stop after passing crosslines
#define fl 2 // robot stop after passing cross line on the left
#define fr 3 // robot stop after passing cross line on the right
#define ll 4 // robot turn left after passing cross line on the left
#define rr 5 // robot turn right after passing cross line on the right
#define ss 6 // robot stop upon sensing cross lines
#define sl 7 // robot stop upon sensing cross line on the left
#define sr 8 // robot stop upon sensing cross line on the right
// sensor define
#define sensor1 128
#define sensor2 64
#define sensor3 32
#define sensor4 16
#define sensor5 8
#define sensor6 4
#define sensor7 2
#define sensor8 1
/*
 * Pin declaration
 */
#define pwm1 PB8
#define pwm2 PB9
#define pwm3 PB6
#define pwm4 PB7
#define buzz PA8
#define btn1 PB3
#define btn2 PB5
#define btn3 PA15
#define btn4 PB4
/*
 * Defined function
 */
#define delay_ms delay
// fungsi yang nanti isinya programnya pengguna
void mode1();
void mode2();
void mode3();

#define led(numberOfTimes,onTime,offTime) buzzerled(0,1,numberOfTimes,onTime,offTime)
#define buzzer(numberOfTimes,onTime,offTime) buzzerled(1,0,numberOfTimes,onTime,offTime)
void buzzerled(bool isUseBuzzer, bool isUseLed, uint8_t numberOfTimes, uint16_t interval, uint16_t customOffTime);
void logPrint(String newLog);
void start(bool useBuzzer, uint16_t pc);
void resetTimer();
void pctimer(uint16_t pc);
void linecolor(bool color);
void sensor(bool dir);
void error(bool useError, uint16_t time);
void controller(float p1, float p2, float p3);
void ffspeed(uint8_t l, uint8_t r);
void bbspeed(uint8_t l, uint8_t r);
void linetrack(uint8_t l);
void motor(int8_t leftSpeed, int8_t rightSpeed, uint16_t runTime);
void readSensor(bool wichSensor, bool debug);
void senData2Bin(uint8_t *line);
void errorRaised();
void controllerRun(uint8_t line, int8_t speed, bool useError);
void backBrakeTimeF(uint8_t speed, int16_t brakeTime);
void runAndDetect(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime);
void line(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime);
void linet(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime);
void timeline(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t totalActionTime);

void turn(bool dir, uint8_t sensor, uint8_t speed, uint8_t backBrakeTime);

void left(uint8_t speed, uint8_t backBrakeTime);
void left1(uint8_t speed, uint8_t backBrakeTime);
void left2(uint8_t speed, uint8_t backBrakeTime);
void left3(uint8_t speed, uint8_t backBrakeTime);
void left4(uint8_t speed, uint8_t backBrakeTime);
void left5(uint8_t speed, uint8_t backBrakeTime);

void right(uint8_t speed, uint8_t backBrakeTime);
void right8(uint8_t speed, uint8_t backBrakeTime);
void right7(uint8_t speed, uint8_t backBrakeTime);
void right6(uint8_t speed, uint8_t backBrakeTime);
void right5(uint8_t speed, uint8_t backBrakeTime);
void right4(uint8_t speed, uint8_t backBrakeTime);

void exline(int8_t leftMotorSpeed, int8_t rightMotorSpeed, uint8_t sensor, int16_t backBrakeTime);
void exturn(int8_t leftMotorSpeed, int8_t rightMotorSpeed, uint8_t sensor, int16_t backBrakeTime);

void linedelay(uint8_t speed, uint16_t runTime, int16_t backBrakeTime);
void linefind(int8_t leftSpeed, int8_t rightSpeed, uint16_t timeToDisregardLine);
void linedline(uint16_t runTime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t brakeTime);
void linetline(uint16_t runTime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t brakeTime);

void sline(uint8_t sensor, uint8_t speed, int16_t backBrakeTime);
void lostline(uint16_t lostLineTime, uint8_t speed, uint16_t runTime, int16_t backBrakeTime);

bool isBtn1();
bool isBtn2();
bool isBtn3();
bool isBtn4();

void printDisplayHome(String text);
void testSensor(bool wichSensor);
void saveSensor(bool wichSensor);
void calibSensor(bool wichSensor);

void showIcon(String filename);
void printMenu(String *menuArray, uint8_t index, uint8_t totalSize);
void showMenu(int8_t *menu0, int8_t *menu1, int8_t *menu2);

void setup();
void loop();
#endif