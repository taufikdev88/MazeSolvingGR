#ifndef MAZESOLVINGGR_H_
#define MAZESOLVINGGR_H_
/*
 * Dependencies library
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <GyverOLED.h>

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
#define pp  pp0
#define tt  tt2
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
#define sensor1   512
#define sensor2   256
#define sensor3   128
#define sensor4   64
#define sensor5   32
#define sensor6   16
#define sensor7   8
#define sensor8   4
#define sensor9   2
#define sensor10  1

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
#define delay_ms delay_maze

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
void motorController(float customKp, float customKi, float customKd);

#define pid1() controller(1.0,10.0)
#define pid2() controller(1.0,20.0)
#define pid3() controller(1.0,30.0)
#define pid4() controller(2.0,20.0)
#define pid5() controller(2.0,40.0)
#define pid6() controller(2.0,60.0)
#define pid7() controller(3.0,30.0)
#define motorPid1() motorController(0.10,0.005,0.01)
#define motorPid2() motorController(0.11,0.002,0.01)
#define motorPid3() motorController(0.09,0.003,0.01)
#define motorPid4() motorController(0.08,0.001,0.01)
#define motorPid5() motorController(0.07,0.002,0.01)
#define motorPid6() motorController(0.06,0.003,0.01)
#define motorPid7() motorController(0.05,0.001,0.01)

void step();
void setstepdelay(uint16_t time);

void ffspeed(uint8_t l, uint8_t r);
void bbspeed(uint8_t l, uint8_t r);

void motor(int16_t leftSpeed, int16_t rightSpeed, uint16_t runTime);
void motorcm(int16_t speed, uint16_t cm, uint16_t backbraketime);
void motorcmavoider(int16_t speed, uint16_t cm, uint16_t backbraketime);
void motorrpm(uint16_t speed, uint16_t runTime, uint16_t backbraketime);
void motorrpmavoider(uint16_t speed, uint16_t runtime, uint16_t backbraketime);
int8_t motorrpmdetectcolor(uint16_t rpmSpeed, uint16_t runTime, bool avoidActive, int8_t colorId);
String motorrpmdetectqr(uint16_t rpmSpeed, uint16_t runTime, bool avoidActive);

void motorsideavoider(int16_t speed, uint16_t cm, uint16_t backbraketime); // not implemented
void motorheading(int16_t speed, float headingRef, uint16_t cm, uint16_t threshold); // not implemented

void line(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime);
void linet(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime, uint16_t actionTime);
void timeline(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime, uint16_t totalActionTime);

void linedelay(int16_t speed, uint16_t runTime, int16_t backbraketime);
void linefind(int16_t leftSpeed, int16_t rightSpeed, uint16_t timeToDisregardLine);
void linedline(uint16_t runTime, uint16_t startSpeed, uint16_t method, uint16_t dir, uint16_t endSpeed, int16_t backbraketime);
void linetline(uint16_t runTime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t backbraketime);

void sline(uint16_t sensor, int16_t speed, int16_t backbraketime);
void slineright(uint16_t sensor, int16_t speed, int16_t backbraketime);
void slineleft(uint16_t sensor, int16_t speed, int16_t backbraketime);
void lostline(uint16_t lostLineTime, int16_t speed, uint16_t runTime, int16_t backbraketime);

void leftline(int16_t speed, uint16_t runtime, bool supermode = false);
String leftlinedetectqr(int16_t speed, uint16_t runtime, bool supermode = false);
void rightline(int16_t speed, uint16_t runtime, bool supermode = false);

void left(int16_t speed, uint16_t backbraketime);
void left1(int16_t speed, uint16_t backbraketime);
void left2(int16_t speed, uint16_t backbraketime);
void left3(int16_t speed, uint16_t backbraketime);
void left4(int16_t speed, uint16_t backbraketime);
void left5(int16_t speed, uint16_t backbraketime);
void left6(int16_t speed, uint16_t backbraketime);
void left7(int16_t speed, uint16_t backbraketime);
void leftenc(int16_t speed, uint16_t count, uint16_t backbraketime);

void right(int16_t speed, uint16_t backbraketime);
void right10(int16_t speed, uint16_t backbraketime);
void right9(int16_t speed, uint16_t backbraketime);
void right8(int16_t speed, uint16_t backbraketime);
void right7(int16_t speed, uint16_t backbraketime);
void right6(int16_t speed, uint16_t backbraketime);
void right5(int16_t speed, uint16_t backbraketime);
void right4(int16_t speed, uint16_t backbraketime);
void rightenc(int16_t speed, uint16_t count, uint16_t backbraketime);

void turnangle(int16_t angle);

void exline(int16_t leftMotorSpeed, int16_t rightMotorSpeed, uint8_t sensor, int16_t backbraketime);
void exturn(int16_t leftMotorSpeed, int16_t rightMotorSpeed, uint8_t sensor, int16_t backbraketime);

void servo(uint8_t pin, uint16_t deg);
void pickdn(uint16_t timedelay);
void placedn(uint16_t timedelay);
void pickup(uint16_t timedelay);
void placeup(uint16_t timedelay);
void take(uint16_t timedelay);
void put(uint16_t timedelay);
void camright(uint16_t timedelay);
void camfront(uint16_t timedelay);
void camleft(uint16_t timedelay);

// ws -> which sensor (ff/bb) ff=front, bb=back 
uint8_t camdetectcolor(bool ws);
String gm66detectqr(bool ws, int8_t t);
String raspidetectqr(bool ws, int8_t t);
void showonlcd(String data);
void delay_maze(uint32_t delayMilli);

#endif