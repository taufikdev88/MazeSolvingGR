#include "MazeSolvingGR.h"

void mode1(){
  start(1,100); // mulai robot dengan menggunakan buzzer dan set pcTime ke 100ms, bisa diatur sesuai kebutuhan
  error(1,200); // gunakan fungsi error agar saat robot keluar jalur lebih dari 200ms, robot akan berhenti
  linecolor(black); // atur robot untuk memulai dengan garis berwarna hitam
  linetrack(2); // atur lebar line ke 2 sensor
  controller(-1,-1,-1); // kp, ki dan kd default untuk sensor
  // setup awal selesai
  sensor(ff);
  line(pp,ff,10,0);

mode1_1: // tambahkan kode dibawah mode1_1 untuk menandai bahwa robot telah melewati setpoint 1 mode 1, setpoint bisa diatur sampai mode1_20
mode1_2:
mode1_3:
mode1_4:
  buzzer(3,200,200); // tandai kalau robot telah berhenti dengan buzzer, bisa diganti seperlunya  
};

void mode2(){
  start(1,100);
  linecolor(white);

mode2_1: // tambahkan kode dibawah mode2_1 untuk menandai bahwa robot telah melewati setpoint 1 mode 2
mode2_2:
mode2_3:
mode2_4:
  buzzer(3,200,200); // tandai kalau robot telah berhenti dengan buzzer, bisa diganti seperlunya
};

void mode3(){
  start(1,100);
  linecolor(black);
  
mode3_1: // tambahkan kode dibawah mode1_1 untuk menandai bahwa robot telah melewati setpoint 1 mode 1
mode3_2:
mode3_3:
mode3_4:
  buzzer(3,200,200); // tandai kalau robot telah berhenti dengan buzzer, bisa diganti seperlunya
};