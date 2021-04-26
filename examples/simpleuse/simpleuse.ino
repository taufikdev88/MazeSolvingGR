#include "MazeSolvingGR.h"

void initMazeRobot(){ // fungsi untuk inisialisasi 
  start(1,100); // mulai robot dengan menggunakan buzzer dan set pcTime ke 100ms, bisa diatur sesuai kebutuhan
  error(1,200); // gunakan fungsi error agar saat robot keluar jalur lebih dari 200ms, robot akan berhenti
  
  // fungsi dibawah ini bisa diatur di dalam mode1,mode2 atau mode3
  linetrack(2); // atur lebar line ke 2 sensor
  sensor(ff); // gunakan sensor depan
  linecolor(black); // atur robot untuk memulai dengan garis berwarna hitam
  controller(-1,-1,-1); // kp, ki dan kd default untuk sensor garis
}

void mode1(){
  step(); // setpoint 0 -> garis start
  linefind(5,5,3000);
  line(pp,ff,10,0);
  step(); // setpoint 1 
  line(pp,ff,11,0);
  step(); // setpoint 2
  line(pp,ff,12,0);
  buzzer(3,200,200); // tandai kalau robot telah berhenti dengan buzzer, bisa diganti seperlunya  
};

void mode2(){
  step();
  lostline(100,10,10,0);
  step();
  line(pp,ff,11,0);
};

void mode3(){

};