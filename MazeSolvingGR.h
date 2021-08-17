#ifndef MAZESOLVINGGR_H_
#define MAZESOLVINGGR_H_
/*
 * Dependencies library
 */
// #include <SPI.h>
// #include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include <MPU6050_Maze.h>
#include <Adafruit_PWMServoDriver.h>

/*
 * Global name
 */
// line color
#define black 0
#define white 1
// run forward / backward
#define ff 0
#define bb 1
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
#define sensor1 512
#define sensor2 256
#define sensor3 128
#define sensor4 64
#define sensor5 32
#define sensor6 16
#define sensor7 8
#define sensor8 4
#define sensor9 2
#define sensor10 1


/*
 * Defined function
 * -> fungsi yang akan diisi oleh pengguna
 */
#define initmazerobot initMazeRobot
void initMazeRobot();
void mode1();
void mode2();
void mode3();

/*
 * Stock functions
 */
#define delay_ms delay
void buzzerled(bool useBuzzer, bool useLED, uint8_t numberOfTimes, uint16_t interval, uint16_t customOffTime = 0);
#define led(numberOfTimes,onTime,offTime) buzzerled(0,1,numberOfTimes,onTime,offTime)
#define buzzer(numberOfTimes,onTime,offTime) buzzerled(1,0,numberOfTimes,onTime,offTime)

void start(bool useBuzzer, uint16_t pc);
void resettimer();
void pctimer(uint16_t pc);
void linecolor(bool color);
void sensor(bool dir);
void error(bool useError, uint16_t time);
void controller(float customKp, float customKd);
#define pid1() controller(1.0,10.0);
#define pid2() controller(1.0,20.0);
#define pid3() controller(1.0,30.0);
#define pid4() controller(2.0,20.0);
#define pid5() controller(2.0,40.0);
#define pid6() controller(2.0,60.0);
#define pid7() controller(3.0,30.0);
void step();
void setstepdelay(uint16_t time);

void ffspeed(uint8_t l, uint8_t r);
void bbspeed(uint8_t l, uint8_t r);

void motor(int8_t leftSpeed, int8_t rightSpeed, uint16_t runTime);
void line(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime);
void linet(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t actionTime);
void timeline(uint8_t method, uint8_t dir, uint8_t speed, int16_t brakeTime, uint16_t totalActionTime);

void left(uint8_t speed, uint8_t backBrakeTime);
void left1(uint8_t speed, uint8_t backBrakeTime);
void left2(uint8_t speed, uint8_t backBrakeTime);
void left3(uint8_t speed, uint8_t backBrakeTime);
void left4(uint8_t speed, uint8_t backBrakeTime);
void left5(uint8_t speed, uint8_t backBrakeTime);
void left6(uint8_t speed, uint8_t backBrakeTime);
void left7(uint8_t speed, uint8_t backBrakeTime);

void right(uint8_t speed, uint8_t backBrakeTime);
void right10(uint8_t speed, uint8_t backBrakeTime);
void right9(uint8_t speed, uint8_t backBrakeTime);
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

void leftline(uint8_t speed, uint16_t runtime);
void rightline(uint8_t speed, uint16_t runtime);

void leftenc(uint8_t speed, uint16_t count, uint8_t backBrakeTime);
void rightenc(uint8_t speed, uint16_t count, uint8_t backBrakeTime);

void turnangle(int16_t angle);
void servo(uint8_t pin, uint16_t deg);

#endif