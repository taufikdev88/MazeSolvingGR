#include "MazeSolvingGR.h"
/*
 * String format addressing and length
 * nama nama fungsi disimpan di eeprom, hanya disimpan alamat eepromnya dan panjang karakter
 * hanya di ambil saat user memilih debug: byfunc / bystep
 */
struct StrFormat
{
  uint32_t startAddress;
  uint8_t length;
};
const StrFormat MenuCalibNew{0, 19}; // 1
const StrFormat MenuCalibTest{19, 20};
const StrFormat MenuCalibSave{39, 20};
const StrFormat MenuFrontSensor{59, 16};
const StrFormat MenuRearSensor{75, 15};
const StrFormat MenuExit{90, 11};
const StrFormat MenuMode{101, 11};
const StrFormat MenuStep{112, 11};
const StrFormat MenuDebug{123, 11};
const StrFormat MenuNext{134, 11};
const StrFormat MenuBack{145, 11}; // 11
const StrFormat MenuMirror{156, 10};
const StrFormat MenuNormal{166, 10};
const StrFormat MenuInfoMode{176, 7};
const StrFormat MenuInfoStep{183, 7};
const StrFormat MenuInfoDebug{190, 7};
const StrFormat InfoNoDebug{197, 8};
const StrFormat InfoByFunc{205, 7};
const StrFormat InfoByStep{212, 7};
const StrFormat InfoMirror{219, 6};
const StrFormat InfoNormal{225, 6}; // 21
const StrFormat InfoCountError{231, 11};
const StrFormat InfoSensorNotDetected{242, 23};
const StrFormat InfoSensorWrong{265, 12};
const StrFormat InfoSaveSuccess{277, 18};
const StrFormat InfoSaveFail{295, 15};
const StrFormat InfoCalibTimeout{310, 17};
const StrFormat InfoCalibFinish{327, 17};
const StrFormat InfoCalibFail{344, 15};
const StrFormat InfoLineBlack{359, 5};
const StrFormat InfoLineWhite{364, 5}; // 31
const StrFormat InfoFF{369, 2};
const StrFormat InfoBB{371, 2};
const StrFormat InfoSdCardError{373, 16};
const StrFormat InfoDelimiter{389, 21};
const StrFormat FuncBuzzerLed{410, 30};
const StrFormat FuncResetTimer{440, 17};
const StrFormat FuncPcTimer{457, 16};
const StrFormat FuncLineColor{473, 18};
const StrFormat FuncSensor{491, 15};
const StrFormat FuncError{506, 17}; // 41
const StrFormat FuncController{523, 22};
const StrFormat FuncFfSpeed{545, 19};
const StrFormat FuncBbSpeed{564, 19};
const StrFormat FuncLineTrack{583, 18};
const StrFormat FuncMotor{601, 20};
const StrFormat FuncLine{621, 22};
const StrFormat FuncLineT{643, 26};
const StrFormat FuncTimeline{669, 29};
const StrFormat FuncLeft{698, 16};
const StrFormat FuncLeft1{714, 17}; // 51
const StrFormat FuncLeft2{731, 17};
const StrFormat FuncLeft3{748, 17};
const StrFormat FuncLeft4{765, 17};
const StrFormat FuncLeft5{782, 17};
const StrFormat FuncLeft6{799, 17};
const StrFormat FuncLeft7{816, 17};
const StrFormat FuncRight{833, 17};
const StrFormat FuncRight10{850, 19};
const StrFormat FuncRight9{869, 18};
const StrFormat FuncRight8{887, 18}; // 61
const StrFormat FuncRight7{905, 18};
const StrFormat FuncRight6{923, 18};
const StrFormat FuncRight5{941, 18};
const StrFormat FuncRight4{959, 18};
const StrFormat FuncExline{977, 24};
const StrFormat FuncExturn{1001, 24};
const StrFormat FuncLineDelay{1025, 24};
const StrFormat FuncLineFind{1049, 23};
const StrFormat FuncLineDLine{1072, 33};
const StrFormat FuncLineTLine{1105, 33}; // 71
const StrFormat FuncSLine{1138, 20};
const StrFormat FuncLostLine{1158, 26};
const StrFormat InfoFrontSensor{1184, 24};
const StrFormat InfoRearSensor{1208, 24};
const StrFormat InfoLineSensor{1232, 20};
const StrFormat InfoStep{1252, 7};
const StrFormat InfoTimeSpent{1259, 13};
const StrFormat InfoYourTime{1272, 13};
const StrFormat InfoLineMissing{1285, 12};

/*
 * EEPROM ADDRESSING
 */
// alamat eeprom untuk menyimpan mode terakhir yang digunakan user
#define ModeAddr 0x10000
// pinout di arduino untuk input output
#define pwm1 PB8
#define pwm2 PB9
#define pwm3 PB0
#define pwm4 PB1
#define buzz PA8
#define btn1 PA15
#define btn2 PB3
#define btn3 PB5
#define btn4 PB4
// aliasing / mirip seperti global variable
#define delaybuzzer 50
#define delaybutton 200
#define fsensor Serial1
#define rsensor Serial2
#define imuserial Serial3
#define none -1
#define centerSide 0
#define leftSide 1
#define rightSide 2
#define sideAvoidance 0
// variable global untuk menyimpan konfig user
bool iUB = true;  // is use buzzer, apakah user mau menggunakan buzzer
bool iUE = true;  // is use error, apakah user mau menggunakan mode error
bool iTF = true;  // is trace forward, apakah mode yang dipilih user adalah maju / mundur
bool iED = false; // is error detected, menyimpan apakah robot mendeteksi error(robot tidak mendeteksi garis/keluar dari garis)
bool iMC = true;  // is mode count, ini untuk setup robot pertama kali untuk menghitung jumlah step yang disetting user
// variable penyimpan jumlah fungsi, jumlah setpoin 
int16_t nFunc = 0;
int16_t nStep = 0;
int16_t mStep = 0;
int8_t runMode = 1; // variable penyimpan mode yang digunakan user
int8_t debugMode = 0; // variable penyimpan mode debug yg dipakai user, apakah no_debug/by_func/by_step
#define no_debug 0
#define by_func 1
#define by_step 2
// variable penyimpang timing
unsigned long errorStart = 0; // menyimpan waktu pertama kali robot tidak mendeteksi garis
// todo: tambahi logika penyimpan start time ke eeprom agar tidak menghitung ulang jika user langsung memilih setpoin selain 0
// todo: tambah comment
unsigned long startTime = 0; // menyimpan robot mulai jalan untuk dihitung berapa lama robot dari start menuju finish
unsigned long btnTiming = 0; // debounce button agar tidak mencet terus menerus
uint16_t maxErrorTime = 100; // flag untuk mengatur lamanya robot menuju error saat tidak mendeteksi garis
uint16_t pcTime = 100;       // flag untuk timer percabangan
uint16_t stepDelayTime = 0;  // timer untuk delay dari step menuju fungsi berikutnya

float Kp = 2.0; // Kp pid sensor garis
float Kd = 0.0; // Kd pid sensor garis
float dError = 0.0; // penyimpan error d sensor garis

float KpMotor = 0.10; // Kp pid kontrol kecepatan motor
float KiMotor = 0.005; // Ki pid kontrol kecepatan motor
float KdMotor = 0.01; // Kd pid kontrol kecepatan motor

uint8_t leftPwmOffsetFwd = 0; // menyimpan offset pwm motor kiri saat maju 
uint8_t rightPwmOffsetFwd = 0; // menyimpan offset pwm motor kanan saat maju
uint8_t leftPwmOffsetBwd = 0; // menyimpan offset pwm motor kiri saat mundur
uint8_t rightPwmOffsetBwd = 0; // menyimpan offset pwm motor kanan saat mundur

float heading = 0; // variable penyiman sudut heading robot  

bool senData[10] = {0}; // variable penyimpan nilai sensor
String mazeLog[2]; // variable penyimpan log fungsi, hanya digunakan saat mode debug by_func/by_step
struct PacketHusky // struktur paket yang diterima dari fungsi parsing data dari husky lens
{
  uint16_t xa = 0;
  uint16_t xb = 0;
  uint16_t ya = 0;
  uint16_t yb = 0;
  int8_t id = 0;
};
struct PacketRaspi // struktur paket yang diterima dari fungsi parsing data dari raspberry pi
{
  uint16_t x = 0;
  uint16_t y = 0;
  uint16_t w = 0;
  uint16_t h = 0;
  String data = "";
};

Adafruit_PWMServoDriver srv = Adafruit_PWMServoDriver(); // objek untuk mengakses servo driver
/*
 * objek untuk mengakses driver oled
 */
// GyverOLED<SSD1306_128x64, OLED_BUFFER> display;
// GyverOLED<SSD1306_128x64, OLED_BUFFER, OLED_SPI, 8, 7, 6> display;
GyverOLED<SSH1106_128x64> display;
// atau bisa passing alamat: display(0x3C);

// **************************************************************************** interrupt service
volatile uint32_t counterL = 0; // variable untuk menampung jumlah interrupt yang dihasilkan oleh encoder motor kiri
volatile uint32_t counterR = 0; // variable untuk menampung jumlah interrupt yang dihasilkan oleh encoder motor kanan
unsigned long lastTimeL = 0; // variable untuk menyimpan waktu terakhir dilakukan penghitungan rpm kiri
unsigned long lastTimeR = 0; // variable untuk menyimpan waktu terakhir dilakukan penghitungan rpm kanan
void pulseCountL() // fungsi yang dijalankan saat interrupt motor kiri aktif
{
  counterL++;
}
void pulseCountR() // fungsi yang dijalankan saat interrupt motor kanan aktif , fungsi ini diinisialisasi di void setup
{
  counterR++;
}
/*
 * fungsi menghitung rpm motor kiri
 */
float getRPML()
{
  detachInterrupt(digitalPinToInterrupt(PA0));
  unsigned long dtime = ((unsigned long)millis() - lastTimeL);
  float rpm = (counterL / 214.0) * (60000.0 / dtime);
  lastTimeL = millis();
  counterL = 0;
  attachInterrupt(digitalPinToInterrupt(PA0), pulseCountL, RISING);
  return rpm;
}
/*
 * fungsi menghitung rpm motor kanan
 */
float getRPMR()
{
  detachInterrupt(digitalPinToInterrupt(PA1));
  unsigned long dtime = ((unsigned long)millis() - lastTimeR);
  float rpm = (counterR / 214.0) * (60000.0 / dtime);
  lastTimeR = millis();
  counterR = 0;
  attachInterrupt(digitalPinToInterrupt(PA1), pulseCountR, RISING);
  return rpm;
}
/*
 * fungsi untuk membersihkan buffer yang ada di serial biar tidak terjadi kesalahan parsing untuk fungsi yang akan dijalankan
 */
void cleanSensor()
{
  while (fsensor.available())
  {
    fsensor.read();
  }
  while (rsensor.available())
  {
    rsensor.read();
  }
}
// **********************************************************************************************
// *************************************************************** communicate with sensor module
/*
 * fungsi yang digunakan untuk update variable heading robot dengan membaca data imu
 */
void imusense()
{
  // not implemented
}
/*
 * fungsi parsing paket husky
 * 
 * contoh penggunaan:
 * siapkan variable paket dahulu untuk menampung hasil parsingan dan isi dengan nilai yang dihasilkan oleh fungsi ini
 * ws -> which sensor (ff/bb) ff=front, bb=back
 * md -> mode (C/Q/L) C=color, Q=QrCode, L=Line
 * 
 * PacketHusky paket = readHusky(ff, C)
 */
PacketHusky readHusky(bool ws, char md)
{
  HardwareSerial *os = (ws == ff ? &fsensor : &rsensor);
  bool ef = false;
  PacketHusky p;
  uint8_t pidx = 0;
  uint8_t pcnt = 0;
  String pstr = "";
  cleanSensor();
  os->write(md);
  unsigned long timestart = millis();
  bool rto = false;
  while (!os->available())
  {
    if ((unsigned long)millis() - timestart > 1000)
    {
      rto = true;
      break;
    }
  }
  if (rto)
    return p;
  while (os->available())
  {
    char c = (char)os->read();
    if (pcnt == 0)
    {
      if (c == 'H')
      {
        ef = false;
      }
      else if (c == 'N')
      {
        pcnt = 0;
        continue;
      }
      else if (c == 'F')
      {
        ef = true;
      }
      else
      {
        pcnt = 0;
        continue;
      }
    }
    else if (pcnt == 1)
    {
      if (ef)
      {
        pcnt = 0;
        continue;
      }
      pidx = 0;
      pstr = "";
    }
    else
    {
      if (c == ',' || c == 'T')
      {
        if (pidx == 0)
        {
          p.xa = pstr.toInt();
        }
        else if (pidx == 1)
        {
          p.ya = pstr.toInt();
        }
        else if (pidx == 2)
        {
          p.xb = pstr.toInt();
        }
        else if (pidx == 3)
        {
          p.yb = pstr.toInt();
        }
        else if (pidx == 4)
        {
          p.id = pstr.toInt();
          pcnt = 0;
          continue;
        }
        pidx++;
        pstr = "";
      }
      else if (isDigit(c))
      {
        pstr += c;
      }
      else
      {
        p.xa = 0;
        p.xb = 0;
        p.ya = 0;
        p.yb = 0;
        p.id = 0;
        pcnt = 0;
        pstr = "";
        continue;
      }
    }
    pcnt++;
    delay(1);
  }
  return p;
}
/*
 * Fungsi yang digunakan untuk membaca paket dari raspberry pi
 *
 * ws -> which sensor (ff/bb) ff=front, bb=back
 * timeout -> isi berapa mili detik maksimal menunggu paket datang dari raspberry pi, jika timeout tercapai sebelum menerima paket maka isinya akan 0 semua
 * 
 * PacketRaspi paket = readRaspi(ff)
 * PacketRaspi packetrasp = readRaspi(ff, 2000)
 */
PacketRaspi readRaspi(bool ws, uint16_t timeout = 1100)
{
  HardwareSerial *os = (ws == ff ? &fsensor : &rsensor);
  PacketRaspi p;
  uint8_t pidx = 0;
  uint8_t pcnt = 0;
  String pstr = "";
  cleanSensor();
  os->write('R');
  unsigned long timestart = millis();
  bool rto = false;
  while (!os->available())
  {
    if ((unsigned long) millis()-timestart > timeout) 
    {
      rto = true;
      break;
    }
  }
  if (rto) return p;
  while (os->available())
  {
    char c = (char) os->read();
    if (pcnt == 0)
    {
      if (c != 'H')
      {
        pcnt = 0;
        continue;
      }
      pstr = "";
    }
    else
    {
      if (c == ',' || c == 'T')
      {
        if (pidx == 0)
        {
          p.x = pstr.toInt();
        }
        else if (pidx == 1)
        {
          p.y = pstr.toInt();
        }
        else if (pidx == 2)
        {
          p.w = pstr.toInt();
        }
        else if (pidx == 3)
        {
          p.h = pstr.toInt();
        }
        else if (pidx == 4)
        {
          p.data = pstr;
          pcnt = 0;
          continue;
        }
        pidx++;
        pstr = "";
      }
      else 
      {
        if (pidx < 4 && !isDigit(c))
        {
          p.x = 0;
          p.y = 0;
          p.w = 0;
          p.h = 0;
          p.data = "";
          pcnt = 0;
          continue;
        }
        else if(pidx == 4 && isDigit(c))
        {
          p.x = 0;
          p.y = 0;
          p.w = 0;
          p.h = 0;
          p.data = "";
          pcnt = 0;
          continue;
        }
        pstr += c;
      }
    }
    pcnt++;
    delay(1);
  }
  return p;
}
/*
 * fungsi untuk membaca nilai sensor, senData[] akan terupdate sesuai dengan kondisi sensor
 * untuk merubah senData[] yang berupa array ke tipe data yang bisa di switch case maka gunakan fungsi senData2Bin dengan menyiapkan variable
 * penampung, contoh:
 * 
 * readSensor(ff); // membaca sensor depan dan otomatis variable senData akan terupdate
 * uint16_t line; // menyiapkan variable penampung data biner untuk dilakukan switch case
 * senData2Bin(&line); // memasukkan array senData ke variable line 
 * 
 * setelah fungsi diatas bisa digunakan untuk switch case kondisi sensor
 * switch(line){
 *  case 0b000011111: error = 1; break;
 * }
 * 
 */
void readSensor(bool ws)
{
  HardwareSerial *os = (ws == ff ? &fsensor : &rsensor);
  bool t[10] = {0};
  os->write('A');
  bool scs = false;
  unsigned long timestart = millis();
  for (uint8_t i = 0; i < 12; i++)
  {
    while (!os->available())
    {
      if ((unsigned long)millis() - timestart > 10)
        break;
    }
    char c = (char)os->read();
    if (i == 0)
    {
      if (c != 'H')
      {
        break;
      }
    }
    else if (i > 0 && i < 11)
    {
      if (!isDigit(c))
      {
        break;
      }
      t[i - 1] = (c == '1' ? 1 : 0);
    }
    else if (i == 11)
    {
      if (c != 'T')
      {
        break;
      }
      for (i = 0; i < 10; i++)
      {
        senData[i] = t[i];
      }
      i = 11;
      scs = true;
    }
  }
  if (!scs)
  {
    for (uint8_t i = 0; i < 10; i++)
    {
      senData[i] = 0;
    }
  }
}
// **********************************************************************************************
// **************************************************************************** board interaction
/*
 * fungsi membaca input button dari board , gunakan shortcut isBtn1() untuk membaca button 1 atau kalau pengen sendiri membaca tombol di
 * pin yang diinginkan gunakan fungsi readBtn, contoh:
 * 
 * if(readBtn(15)){
 *  kinematik(100,100);
 * }
 * if(isBtn1()){
 *  kinematik(50,50);
 * }
 */
#define isBtn1() readBtn(btn1)
#define isBtn2() readBtn(btn2)
#define isBtn3() readBtn(btn3)
#define isBtn4() readBtn(btn4)
bool readBtn(uint8_t p)
{
  uint8_t c = 0;
  for (uint8_t i = 0; i < 7; i++)
  {
    if (digitalRead(p) == 0)
    {
      c++;
    }
    delayMicroseconds(10);
  }
  if (c > 4)
  {
    return true;
  }
  return false;
}
/*
 * gunakan fungsi ini disetiap akhir fungsi yang dijalankan oleh user, yang menandakan pergantian fungsi
 * fungsi ini sudah dilengkapi dengan pengecekan apakah user mengaktifkan buzzer / mematikan buzzer
 * 
 * usebuzzeron();
 * *beberapa fungsi didalam sini 
 * useBuzzerOff();
 */
void usebuzzeron()
{
  if (iUB)
  {
    digitalWrite(buzz, 1);
    digitalWrite(LED_BUILTIN, 1);
  }
}
void useBuzzerOff()
{
  if (iUB)
  {
    digitalWrite(buzz, 0);
    digitalWrite(LED_BUILTIN, 0);
  }
}
/*
 * perlakuan khusus untuk button 4 / button yang kami anggap sebagai button go, saat ini digunakan untuk menunggu user memencet tombol go
 * saat user menggunakan mode debug by_func/by_step
 */
void waitKey4()
{
  if (debugMode == by_func)
  {
    while (!isBtn4() || (unsigned long)millis() - btnTiming <= delaybutton)
    {
    };
    btnTiming = millis();
    digitalWrite(buzz, 1);
    delay(delaybuzzer);
    digitalWrite(buzz, 0);
  }
}
/*
 * menyimpan sebuah nilai byte(8 bit data/dalam desimal adalah 255) ke eeprom
 *
 * a = address, d = data
 */
void writeByteEEPROM(uint32_t a, byte d)
{
  if (a > 0x1FFFE)
  {
    return;
  }
  uint8_t ea = 0x50;
  if (a > 0xFFFF)
  {
    ea = 0x51;
    a -= 0xFFFF;
  }
  Wire.beginTransmission(ea);
  Wire.write((uint8_t)(a >> 8));
  Wire.write((uint8_t)(a & 0xFF));
  Wire.write(d);
  Wire.endTransmission();
}
/*
 * mengambil nilai dari alamat eeprom
 *
 * a = address
 */
byte readByteEEPROM(uint32_t a)
{
  if (a > 0x1FFFE)
  {
    return '?';
  }
  uint8_t ea = 0x50;
  if (a > 0xFFFF)
  {
    ea = 0x51;
    a -= 0xFFFF;
  }
  byte data = '?';
  Wire.beginTransmission(ea);
  Wire.write((uint8_t)(a >> 8));
  Wire.write((uint8_t)(a & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(ea, 1);
  if (Wire.available())
  {
    data = Wire.read();
  }
  return data;
}
/*
 * fungsi untuk mengambil nama fungsi yang tersimpan di eeprom
 * penggunaan fungsi ini berguna agar tidak terlalu banyak mendefinisikan string di dalam flash agar tidak membuang memory flash hanya untuk
 * menyimpan nama fungsi
 * 
 * StrFormat yang bisa digunakan ada di atas sendiri dan akan mengembalikan string yang tersimpan
 * contoh:
 * sprintf(d, getStringFormat(InfoFF).c_str(), senData[0], senData[1], ... ,senData[9]);
 * display.println(d);
 * 
 */
String getStringFormat(StrFormat sf)
{
  String r = "";
  for (uint32_t a = sf.startAddress; a < (sf.startAddress + sf.length); a++)
  {
    r += (char)readByteEEPROM(a);
  }
  return r;
}
/*
 * fungsi untuk menampikan karakter ke display dan tetap menyimpan 2 log terakhir yang dikirim untuk ditampilkan ke user
 */
void log(String l)
{
  display.clear();
  display.setCursorXY(0, 0);
  display.print(runMode);
  display.print(F(" / "));
  switch (debugMode)
  {
  case no_debug:
    display.println(getStringFormat(InfoNoDebug));
    break;
  case by_func:
    display.println(getStringFormat(InfoByFunc));
    break;
  case by_step:
    display.println(getStringFormat(InfoByStep));
    break;
  }
  display.println(getStringFormat(InfoDelimiter));
  display.setCursorXY(0, 16);
  mazeLog[0] = mazeLog[1];
  mazeLog[1] = l;
  display.println(mazeLog[0]);
  display.println(mazeLog[1]);
  display.update();
}
/*
 * Tampilan menu awal untuk user memilih mode, memilih setpoint dan memilih tipe debugging
 */
void displayMenu()
{
  display.clear();
  display.setCursorXY(0, 0);
  display.print(getStringFormat(MenuMode));
  display.println(runMode);
  display.print(getStringFormat(MenuStep));
  display.println(nStep);
  display.print(getStringFormat(MenuDebug));
  switch (debugMode)
  {
  case no_debug:
    display.println(getStringFormat(InfoNoDebug));
    break;
  case by_func:
    display.println(getStringFormat(InfoByFunc));
    break;
  case by_step:
    display.println(getStringFormat(InfoByStep));
    break;
  }
  display.println(getStringFormat(MenuNext));
  char d[InfoFrontSensor.length + 10] = {0};
  sprintf(d, "enc l:%d r:%d", counterL, counterR); // eepromflag
  display.println(d);
  display.setCursorXY(0, 48);
  readSensor(ff);
  sprintf(d, getStringFormat(InfoFrontSensor).c_str(), senData[0], senData[1], senData[2], senData[3], senData[4], senData[5], senData[6], senData[7], senData[8], senData[9]);
  display.println(d);
  readSensor(bb);
  sprintf(d, getStringFormat(InfoRearSensor).c_str(), senData[9], senData[8], senData[7], senData[6], senData[5], senData[4], senData[3], senData[2], senData[1], senData[0]);
  display.println(d);
  display.update();
}
// **********************************************************************************************
// *********************************************************************************** robot data
/*
 * fungsi mengambil mode terakhir yang digunakan oleh user
 */
void getLastMode()
{
  runMode = constrain(readByteEEPROM(ModeAddr), 1, 3);
}
/*
 * fungsi menyimpan mode terakhir yang digunakan oleh user, hanya digunakan di void setup()
 */
void setLastMode()
{
  writeByteEEPROM(ModeAddr, runMode);
}
/*
 * fungsi menghitung total setpoin disetiap mode, hanya digunakan di void setup()
 */
void countSetPoint()
{
  mStep = -1;
  nStep = 0;
  iMC = true;
  switch (runMode)
  {
  case 1:
    mode1();
    break;
  case 2:
    mode2();
    break;
  case 3:
    mode3();
    break;
  }
  iMC = false;
}
/*
 * fungsi mengubah array senData menjadi data yang bisa digunakan untuk switch case
 */
void senData2Bin(uint16_t *l)
{
  if (senData[0]) *l = *l | 0b1000000000;
  if (senData[1]) *l = *l | 0b0100000000;
  if (senData[2]) *l = *l | 0b0010000000;
  if (senData[3]) *l = *l | 0b0001000000;
  if (senData[4]) *l = *l | 0b0000100000;
  if (senData[5]) *l = *l | 0b0000010000;
  if (senData[6]) *l = *l | 0b0000001000;
  if (senData[7]) *l = *l | 0b0000000100;
  if (senData[8]) *l = *l | 0b0000000010;
  if (senData[9]) *l = *l | 0b0000000001;
}
// **********************************************************************************************
// *********************************************************************************** robot move
/*
 * fungsi untuk menggerakkan motor kiri dan kanan
 *
 * ls = leftspeed, rs = rightspeed
 * jika positif motor akan berputar maju
 * jika negatif motor akan berputar mundur
 */
void kinematik(int16_t ls, int16_t rs)
{
  ls = constrain(ls, -255, 255);
  rs = constrain(rs, -255, 255);

  if (ls < 0)
  {
    ls = -ls;
    analogWrite((iTF ? pwm1 : pwm3), 0);
    analogWrite((iTF ? pwm2 : pwm4), ls + leftPwmOffsetBwd);
  }
  else
  {
    analogWrite((iTF ? pwm1 : pwm3), ls + leftPwmOffsetFwd);
    analogWrite((iTF ? pwm2 : pwm4), 0);
  }
  if (rs < 0)
  {
    rs = -rs;
    analogWrite((iTF ? pwm3 : pwm1), 0);
    analogWrite((iTF ? pwm4 : pwm2), rs + rightPwmOffsetBwd);
  }
  else
  {
    analogWrite((iTF ? pwm3 : pwm1), rs + rightPwmOffsetFwd);
    analogWrite((iTF ? pwm4 : pwm2), 0);
  }
}
/*
 * fungsi untuk menahan robot agar berhenti saat robot kehilangan garis
 *
 * fungsi ini hanya aktif jika user mengaktifkan error dan tidak semua fungsi lain menggunakan fungsi ini
 * karena robot juga perlu suatu saat berjalan tanpa melewati garis
 */
void errorRaised()
{
  kinematik(0, 0);
  while (1)
  {
    kinematik(0, 0);
    digitalWrite(buzz, 1);
    readSensor(iTF ? ff : bb);
    uint16_t l = 0x00;
    senData2Bin(&l);
    delay(delaybuzzer);
    digitalWrite(buzz, 0);
    delay(delaybuzzer);
    if (l != 0x00)
      break;
  }
  log(F("run"));
}
/*
 * fungsi untuk injek/mengubah kecepatan motor kiri dan kanan saat mendeteksi garis di sensor
 * gunanya agar robot tetap menghindari garis, contoh:
 * 
 * int16_t *spd = sidelineError();
 * 
 * *(spd) = adalah nilai kecepatan motor kiri yang didapatkan dari sidelineError
 * *(spd+1) = adalah nilai kecepatan motor kanan yang didapatkan dari sidelineError
 */
// error saat menggunakan avoider
int16_t* sidelineError(bool supermode = false){
  int16_t error = 0;
  static int16_t spd[2];
  spd[0] = 0;
  spd[1] = 0;
  readSensor(iTF ? ff : bb);
  uint16_t l = 0;
  senData2Bin(&l);
  bool center = false;
  switch (l)
  {
  case 0b1000000000:
    spd[1] = 21;
    break;
  case 0b1100000000:
    spd[1] = 18;
    break;
  case 0b0100000000:
    spd[1] = 15;
    break;
  case 0b0110000000:
    spd[1] = 12;
    break;
  case 0b0010000000:
    spd[1] = 9;
    break;
  case 0b0011000000:
    spd[1] = 6;
    break;
  case 0b0001000000:
    spd[1] = 3;
    break;
  case 0b0001100000:
    spd[1] = 0;
    center = true;
    break;
  case 0b0000100000:
    spd[1] = 0;
    center = true;
    break;
  case 0b0000010000:
    spd[0] = 0;
    center = true;
    break;
  case 0b0000011000:
    spd[0] = 0;
    center = true;
    break;
  case 0b0000001000:
    spd[0] = 3;
    break;
  case 0b0000001100:
    spd[0] = 6;
    break;
  case 0b0000000100:
    spd[0] = 9;
    break;
  case 0b0000000110:
    spd[0] = 12;
    break;
  case 0b0000000010:
    spd[0] = 15;
    break;
  case 0b0000000011:
    spd[0] = 18;
    break;
  case 0b0000000001:
    spd[0] = 21;
    break;
  case 0b1000000001:
  case 0b1100000001:
  case 0b1000000011:
  case 0b0000110000:
  case 0b0001110000:
  case 0b0000111000:
  case 0b0001111000:
  case 0b0011110000:
  case 0b0000111100:
  case 0b0011111100:
  case 0b0111111000:
  case 0b0001111110:
  case 0b0111111110:
  case 0b1111111110:
  case 0b1111111100:
  case 0b1111111000:
  case 0b1111100000:
  case 0b1111000000:
  case 0b1110000000:
  case 0b0000000111:
  case 0b0000001111:
  case 0b0000011111:
  case 0b0000111111:
  case 0b0001111111:
  case 0b0011111111:
  case 0b0111111111:
  case 0b1111111111:
    center = true;
    break;
  }
  if (center)
  {
    spd[0] = 404;
    spd[1] = 404;
  }
  return spd;
}
/*
 * fungsi untuk mendapatkan nilai error saat mode mengikuti garis dengan referensi sensor paling kiri
 * nilai error ini digunakan untuk input error pid
 */
// follow left line
int16_t leftlineError(bool supermode = false)
{
  int16_t error = 0;
  readSensor(iTF ? ff : bb);
  uint16_t l = 0;
  senData2Bin(&l);
  switch (l)
  {
  // tidak mendeteksi garis
  case 0b0000000000: error = -13; break;
  // sensor 2 garis data bagus, 3
  case 0b1000000000: error = 0; break;
  case 0b1100000000: error = 13; break;
  case 0b0100000000: error = 16; break;
  case 0b0110000000: error = 19; break;
  case 0b0010000000: error = 21; break;
  case 0b0011000000: error = 24; break;
  case 0b0001000000: error = 27; break;
  case 0b0001100000: error = 30; break;
  case 0b0000100000: error = 36; break;
  case 0b0000110000: error = 39; break;
  case 0b0000010000: error = 42; break;
  case 0b0000011000: error = 45; break;
  case 0b0000001000: error = 48; break;
  case 0b0000001100: error = 51; break;
  case 0b0000000100: error = 54; break;
  case 0b0000000110: error = 57; break;
  case 0b0000000010: error = 60; break;
  case 0b0000000011: error = 63; break;
  case 0b0000000001: error = 66; break;
  // sensor 3 garis data bagus, 7
  case 0b1110000000: error = 13; break;
  case 0b0111000000: error = 20; break;
  case 0b0011100000: error = 27; break;
  case 0b0001110000: error = 34; break;
  case 0b0000111000: error = 41; break;
  case 0b0000011100: error = 48; break;
  case 0b0000001110: error = 55; break;
  case 0b0000000111: error = 62; break;
  // sensor 4 garis data bagus, 11
  case 0b1111000000: error = 26; break;
  case 0b0111100000: error = 37; break;
  case 0b0011110000: error = 48; break;
  case 0b0001111000: error = 59; break;
  case 0b0000111100: error = 70; break;
  case 0b0000011110: error = 81; break;
  case 0b0000001111: error = 92; break;
  // sensor 5 garis data bagus, 17
  case 0b1111100000: error = 39; break;
  case 0b0111110000: error = 56; break;
  case 0b0011111000: error = 73; break;
  case 0b0001111100: error = 90; break;
  case 0b0000111110: error = 107; break;
  case 0b0000011111: error = 124; break;
  // sensor 6 garis data bagus, 23
  case 0b1111110000: error = 52; break;
  case 0b0111111000: error = 75; break;
  case 0b0011111100: error = 98; break;
  case 0b0001111110: error = 121; break;
  case 0b0000111111: error = 144; break;
  //  di tempat yg tepat tp sensor lain error
  case 0b1000000001:
  case 0b1000000010:
  case 0b1000000100:
  case 0b1000001000:
  case 0b1000010000:
  case 0b1000100000:
  case 0b1001000000:
  case 0b1010000000:
  case 0b1001000001:
  case 0b1000100001:
  case 0b1000010001:
  case 0b1000001001:
  case 0b1010000010:
  case 0b1001000010:
  case 0b1000100010:
  case 0b1000010010:
  case 0b1000001010:
  case 0b1010000100:
  case 0b1001000100:
  case 0b1000100100:
  case 0b1000010100:
  case 0b1001001000:
  case 0b1000101000: error = 0; break;
  }
  if (supermode)
  {
    switch (l)
    {
    case 0b1111100000: error = 10; break;
    case 0b1111110000: error = 22; break;
    case 0b1111111000: error = 36; break;
    case 0b1111111100: error = 42; break;
    case 0b1111111110: error = 60; break;
    case 0b1111111111: error = 80; break;
    case 0b0111111111: error = 102; break;
    case 0b0011111111: error = 126; break;
    case 0b0001111111: error = 152; break;
    case 0b0000111111: error = 180; break;
    case 0b0000011111: error = 210; break;
    case 0b1100000001: error = 130; break;
    case 0b1100000011: error = 160; break;
    case 0b1100000111: error = 190; break;
    case 0b1100001111: error = 220; break;
    case 0b1100011111: error = 255; break;
    }
  }
  return error;
}
/*
 * fungsi untuk mendapatkan nilai error saat mode mengikuti garis dengan referensi sensor paling kanan
 * nilai error ini digunakan untuk input error pid
 */
int16_t rightlineError(bool supermode = false)
{
  readSensor(iTF ? ff : bb);
  uint16_t l = 0;
  senData2Bin(&l);
  int16_t error = 0;
  switch (l)
  {
  // sensor 2 garis data bagus
  case 0b0000000000: error = 13; break;
  case 0b1000000000: error = -63; break;
  case 0b1100000000: error = -60; break;
  case 0b0100000000: error = -57; break;
  case 0b0110000000: error = -54; break;
  case 0b0010000000: error = -51; break;
  case 0b0011000000: error = -48; break;
  case 0b0001000000: error = -45; break;
  case 0b0001100000: error = -42; break;
  case 0b0000100000: error = -39; break;
  case 0b0000110000: error = -36; break;
  case 0b0000010000: error = -33; break;
  case 0b0000011000: error = -30; break;
  case 0b0000001000: error = -27; break;
  case 0b0000001100: error = -24; break;
  case 0b0000000100: error = -21; break;
  case 0b0000000110: error = -19; break;
  case 0b0000000010: error = -16; break;
  case 0b0000000011: error = -13; break;
  case 0b0000000001: error = 0; break;
  // sensor 3 garis data bagus
  case 0b1110000000: error = -62; break;
  case 0b0111000000: error = -55; break;
  case 0b0011100000: error = -48; break;
  case 0b0001110000: error = -41; break;
  case 0b0000111000: error = -34; break;
  case 0b0000011100: error = -27; break;
  case 0b0000001110: error = -20; break;
  case 0b0000000111: error = -13; break;
  // sensor 4 garis data bagus
  case 0b1111000000: error = -92; break;
  case 0b0111100000: error = -81; break;
  case 0b0011110000: error = -70; break;
  case 0b0001111000: error = -59; break;
  case 0b0000111100: error = -48; break;
  case 0b0000011110: error = -37; break;
  case 0b0000001111: error = -26; break;
  //  di tempat yg tepat tp sensor lain error
  case 0b1000000001:
  case 0b0100000001:
  case 0b0010000001:
  case 0b0001000001:
  case 0b0000100001:
  case 0b0000010001:
  case 0b0000001001:
  case 0b0000000101:
  case 0b1000001001:
  case 0b1000010001:
  case 0b1000100001:
  case 0b1001000001:
  case 0b0100000101:
  case 0b0100001001:
  case 0b0100010001:
  case 0b0100100001:
  case 0b0101000001:
  case 0b0010000101:
  case 0b0010001001:
  case 0b0010010001:
  case 0b0010100001:
  case 0b0001001001:
  case 0b0001010001: error = 0; break;
  }
  if (supermode)
  {
    switch (l)
    {
    case 0b1111100000: error = -210; break;
    case 0b1111110000: error = -180; break;
    case 0b1111111000: error = -152; break;
    case 0b1111111100: error = -126; break;
    case 0b1111111110: error = -102; break;
    case 0b1111111111: error = -80; break;
    case 0b0111111111: error = -60; break;
    case 0b0011111111: error = -42; break;
    case 0b0001111111: error = -36; break;
    case 0b0000111111: error = -22; break;
    case 0b0000011111: error = -10; break;
    case 0b1000000011: error = -130; break;
    case 0b1100000011: error = -160; break;
    case 0b1110000011: error = -190; break;
    case 0b1111000011: error = -220; break;
    case 0b1111100011: error = -250; break;
    }
  }
  return error;
}
/*
 * fungsi ini digunakan untuk menjalankan pid tetapi berbeda dengan controllerRun, jika controllerRun error jika kehilangan garis
 * maka fungsi ini tidak akan menghasilkan error saat tidak mendeteksi garis
 */
void controllerNoErrorRun(int16_t speed, bool supermode, int16_t dir, uint16_t runtime = 1)
{
  unsigned long timestart = millis();
  while ((unsigned long) millis()-timestart <= runtime)
  {
    int16_t pwm = 0;
    int16_t error = 0;
    switch (dir)
    {
      case rightSide: error = rightlineError(supermode); break;
      case leftSide: error = leftlineError(supermode); break;
    }
    pwm = Kp * error + Kd * (error - dError);
    pwm = constrain(pwm, -255, 255);
    dError = error;
    int16_t spdl = speed + pwm;
    int16_t spdr = speed - pwm;
    kinematik((iTF ? spdl : -spdl), (iTF ? spdr : -spdr));
  }
  if (runtime > 1)
    kinematik(0,0);
}
/*
 * fungsi ini akan mengaktifkan pid sensor agar robot mengikuti garis terus menerus
 * fungsi ini digunakan di beberapa fungsi lain seperti detectorRun, lineDelay, dll
 */
void controllerRun(uint16_t line, int16_t speed, bool useError = true)
{
  int16_t pwm = 0;
  int8_t e = 0;
  switch (line)
  {
  case 0b1000000000: iED = false; e = -27; break;
  case 0b1100000000: iED = false; e = -24; break;
  case 0b0100000000: iED = false; e = -21; break;
  case 0b0110000000: iED = false; e = -19; break;
  case 0b0010000000: iED = false; e = -16; break;
  case 0b0011000000: iED = false; e = -5; break;
  case 0b0001000000: iED = false; e = -3; break;
  case 0b0001100000: iED = false; e = -1; break;
  case 0b0000011000: iED = false; e = 1; break;
  case 0b0000001000: iED = false; e = 3; break;
  case 0b0000001100: iED = false; e = 5; break;
  case 0b0000000100: iED = false; e = 16; break;
  case 0b0000000110: iED = false; e = 19; break;
  case 0b0000000010: iED = false; e = 21; break;
  case 0b0000000011: iED = false; e = 24; break;
  case 0b0000000001: iED = false; e = 27; break;
  case 0b1110000000: iED = false; e = -21; break;
  case 0b0111000000: iED = false; e = -14; break;
  case 0b0011100000: iED = false; e = -7; break;
  case 0b0000011100: iED = false; e = 7; break;
  case 0b0000001110: iED = false; e = 14; break;
  case 0b0000000111: iED = false; e = 21; break;
  case 0b1111000000: iED = false; e = -36; break;
  case 0b0111100000: iED = false; e = -28; break;
  case 0b0000011110: iED = false; e = 28; break;
  case 0b0000001111: iED = false; e = 36; break;
  case 0b0000100000: iED = false; e = 0; break;
  case 0b0000110000: iED = false; e = 0; break;
  case 0b0000010000: iED = false; e = 0; break;
  case 0b0001110000: iED = false; e = 0; break;
  case 0b0000111000: iED = false; e = 0; break;
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
  case 0b1000110000: iED = false; e = 0; break;
  // error di sensor lain tp parah
  case 0b0011110000: iED = false; e = -1; break;
  case 0b0000111100: iED = false; e = 1; break;
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
  case 0b0010000100: iED = false; e = 0; break;
  // error sensor kondisi tengah
  case 0b0000100011:
  case 0b0000100100:
  case 0b0000100010:
  case 0b0000100110: iED = false; e = -1; break;
  case 0b0010010000:
  case 0b0100010000:
  case 0b1100010000:
  case 0b1000010000:
  case 0b0110010000: iED = false; e = 1; break;
  // error sensor kondisi samping medium
  case 0b0001100011:
  case 0b0001100100:
  case 0b0001100010:
  case 0b0001100110: iED = false; e = -3; break;
  case 0b1000011000:
  case 0b1100011000:
  case 0b0010011000:
  case 0b0100011000:
  case 0b0110011000: iED = false; e = 3; break;
  // error sensor kondisi samping high
  // case 0b00100010:
  // case 0b00100001:
  // case 0b00100011: e = -5; break;
  // case 0b01000100:
  // case 0b10000100:
  // case 0b11000100: e = 5; break;
  // garis di semua line
  case 0b1111111111: iED = false; e = 0; break;
  case 0b0000000000: e = 0; 
    if (!iED)
    {
      log(getStringFormat(InfoLineMissing));
      errorStart = millis();
      iED = true;
    }
    break;
  }

  if (useError && iED && iUE && (unsigned long)millis() - errorStart >= maxErrorTime)
  {
    kinematik(0, 0);
    errorRaised();
    return;
  }

  pwm = Kp * e + Kd * (e - dError);
  pwm = constrain(pwm, -255, 255);
  dError = e;
  int16_t spdl = speed + pwm;
  int16_t spdr = speed - pwm;
  kinematik((iTF ? spdl : -spdl), (iTF ? spdr : -spdr));
}
/*
 * fungsi ini digunakan untuk robot setelah selesai menjalankan fungsi line, lined, linet, linedelay apakah mau maju sedikit atau mundur sedikit
 * tujuannya adalah untuk mengoreksi posisi robot
 */
void backbraketimeFunc(int16_t speed, int16_t backbraketime, bool useError = true)
{
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < backbraketime)
  {
    kinematik((iTF ? speed : -speed), (iTF ? speed : -speed));
  }
  timeStart = millis();
  while ((unsigned long)millis() - timeStart < -backbraketime)
  {
    readSensor(iTF ? ff : bb);
    uint16_t l = 0x00;
    senData2Bin(&l);
    controllerRun(l, speed, useError);
  }
  kinematik(0, 0);
}
/*
 * fungsi ini adalah fungsi yang menjalankan logika pendeteksian percabangan, pendeteksian pertigaan, perempatan dan implementasi
 * ke arah mana robot akan menghadap setelah mendeteksi percabangan
 * fungsi ini digunakan oleh beberapa fungsi lain seperti lineFunc, linetFunc
 */
void detectorRun(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime, uint16_t actionTime)
{
  unsigned long detectTime1 = millis();
  unsigned long detectTime2 = millis();
  bool detectFlag = false;
  bool crossDetected = false;
  int8_t side = none;
  iED = false;
  cleanSensor();
  unsigned long timeStart = millis();
  while (!crossDetected)
  {
    readSensor(iTF ? ff : bb);
    if (actionTime != (uint16_t)-1)
    {
      if ((unsigned long)millis() - timeStart >= actionTime)
      {
        break;
      }
    }
    switch (method)
    {
    case pp0:
      if (senData[0])
      {
        detectFlag = true;
        side = leftSide;
        detectTime1 = millis();
      }
      else if (side != leftSide && detectFlag)
      {
        detectTime2 = millis();
        if ((detectTime2 - detectTime1) < pcTime)
        {
          crossDetected = true;
        }
        else
        {
          detectFlag = false;
          side = none;
        }
      }
      if (senData[9])
      {
        if (!detectFlag)
        {
          detectFlag = true;
          side = rightSide;
          detectTime1 = millis();
        }
        else if (side != rightSide && detectFlag)
        {
          detectTime2 = millis();
          if ((detectTime2 - detectTime1) < pcTime)
          {
            crossDetected = true;
          }
          else
          {
            detectFlag = false;
            side = none;
          }
        }
      }
      break;
    case pp1:
      if (senData[0] && senData[1])
      {
        if (!detectFlag)
        {
          detectFlag = true;
          side = leftSide;
          detectTime1 = millis();
        }
        else if (side != leftSide && detectFlag)
        {
          detectTime2 = millis();
          if ((detectTime2 - detectTime1) < pcTime)
          {
            crossDetected = true;
          }
          else
          {
            detectFlag = false;
            side = none;
          }
        }
      }
      if (senData[8] && senData[9])
      {
        if (!detectFlag)
        {
          detectFlag = true;
          side = rightSide;
          detectTime1 = millis();
        }
        else if (side != rightSide && detectFlag)
        {
          detectTime2 = millis();
          if ((detectTime2 - detectTime1) < pcTime)
          {
            crossDetected = true;
          }
          else
          {
            detectFlag = false;
            side = none;
          }
        }
      }
      break;
    case pp2:
      side = 0;
      if (senData[0]) ++side;
      if (senData[1]) ++side;
      if (senData[2]) ++side;
      if (senData[7]) ++side;
      if (senData[8]) ++side;
      if (senData[9]) ++side;
      if (side >= 5) crossDetected = true;
      break;
    case pp3:
      side = 0;
      if (senData[0]) ++side;
      if (senData[1]) ++side;
      if (senData[2]) ++side;
      if (senData[7]) ++side;
      if (senData[8]) ++side;
      if (senData[9]) ++side;
      if (side >= 6) crossDetected = true;
      break;
    case tt1:
      if ((dir == fl || dir == ll || dir == sl) && senData[0])
      {
        crossDetected = true;
      }
      else if ((dir == fr || dir == rr || dir == sr) && senData[9])
      {
        crossDetected = true;
      }
      break;
    case tt2:
      if ((dir == fl || dir == ll || dir == sl) && senData[0] && senData[1])
      {
        crossDetected = true;
      }
      else if ((dir == fr || dir == rr || dir == sr) && senData[8] && senData[9])
      {
        crossDetected = true;
      }
      break;
    case tt3:
      if ((dir == fl || dir == ll || dir == sl) && senData[0] && senData[1] && senData[2])
      {
        crossDetected = true;
      }
      else if ((dir == fr || dir == rr || dir == sr) && senData[7] && senData[8] && senData[9])
      {
        crossDetected = true;
      }
      break;
    }
    if (crossDetected)
      continue;
    uint16_t l = 0;
    senData2Bin(&l);
    controllerRun(l, speed);
  }
  switch (dir)
  {
  case ss:
  case sl:
  case sr:
    kinematik(0, 0);
    break;
  case ff:
  case fl:
  case fr:
  case ll:
  case rr:
    readSensor(iTF ? ff : bb);
    while (senData[0] || senData[9])
    {
      if (iTF)
      {
        readSensor(ff);
        kinematik(speed, speed);
      }
      else
      {
        readSensor(bb);
        kinematik(-speed, -speed);
      }
    }
    kinematik(0, 0);
    break;
  }
  usebuzzeron();
  backbraketimeFunc(speed, backbraketime);
  crossDetected = false;
  switch (dir)
  {
  case ll:
    while (!crossDetected || (!senData[4] && !senData[5]))
    {
      if (iTF)
      {
        readSensor(ff);
        kinematik(-speed, speed);
      }
      else
      {
        readSensor(bb);
        kinematik(-speed, speed);
      }
      if (senData[1])
      {
        crossDetected = true;
      }
    }
    break;
  case rr:
    while (!crossDetected || (!senData[4] && !senData[5]))
    {
      if (iTF)
      {
        readSensor(ff);
        kinematik(speed, -speed);
      }
      else
      {
        readSensor(bb);
        kinematik(-speed, speed);
      }
      if (senData[6])
      {
        crossDetected = true;
      }
    }
    break;
  }
  kinematik(0, 0);
  useBuzzerOff();
}
// **********************************************************************************************
// *************************************************************************** interface function
/*
 * fungsi dasar tanpa log dan fungsi ini digunakan oleh beberapa fungsi lain seperti line, timeline, dll
 */
void lineFunc(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime)
{
  detectorRun(method, dir, speed, backbraketime, -1);
}
/*
 * fungsi dasar linet tanpa log, fungsi ini ada karena fungsi ini digunakan oleh beberapa fungsi lain seperti linet, linetline, timeline
 */
void linetFunc(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime, uint16_t actionTime)
{
  detectorRun(method, dir, speed, backbraketime, actionTime);
}
/*
 * fungsi dasar linedelay tanpa log, fungsi ini ada karena fungsi ini digunakan oleh beberapa fungsi lain seperti linedelay, linedline
 */
void linedelayFunc(int16_t speed, uint16_t runtime, int16_t backbraketime)
{
  unsigned long timeStart = millis();
  cleanSensor();
  while ((unsigned long)millis() - timeStart < runtime)
  {
    readSensor(iTF ? ff : bb);
    uint16_t l = 0;
    senData2Bin(&l);
    controllerRun(l, speed);
  }
  usebuzzeron();
  backbraketimeFunc(speed, backbraketime);
  kinematik(0, 0);
  useBuzzerOff();
}
/*
 * fungsi dasar tanpa log, fungsi ini ada karena fungsi ini digunakan oleh beberapa fungsi lain seperti motorcm, motorcmdetectcolor
 */
void motorcmFunc(int16_t speed, uint16_t cm, uint16_t backbraketime, int8_t method = none)
{
  uint32_t step = 15 * cm;
  counterR = 0;
  while (counterR < step)
  {
    if (method == sideAvoidance)
    {
      readSensor(iTF ? ff : bb);
      if (senData[0])
      {
        kinematik(0, (iTF ? speed : -speed));
        continue;
      }
      if (senData[9])
      {
        kinematik((iTF ? speed : -speed), 0);
        continue;
      }
    }

    kinematik((iTF ? speed : -speed), (iTF ? speed : -speed));
  }
  usebuzzeron();
  backbraketimeFunc(speed, backbraketime);
  useBuzzerOff();
}
// **********************************************************************************************
// ********************************************************************************* turning func
/*
 * fungsi dasar agar robot belok kearah yang ditentukan sampai sensor yang digunakan mendeteksi garis, fungsi ini digunalan oleh 
 * beberapa fungsi lain seperti left, left1-left7, right, right10-right4
 */
void turn(bool dir, uint8_t sensor, int16_t speed, uint16_t backbraketime)
{
  readSensor(iTF ? ff : bb);
  while (!senData[sensor])
  {
    readSensor(iTF ? ff : bb);
    switch (dir)
    {
    case leftSide:
      kinematik(-speed, speed);
      break;
    case rightSide:
      kinematik(speed, -speed);
      break;
    }
  }
  kinematik(0, 0);
  usebuzzeron();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < backbraketime)
  {
    switch (dir)
    {
    case leftSide:
      kinematik(speed, -speed);
      break;
    case rightSide:
      kinematik(-speed, speed);
      break;
    }
  }
  kinematik(0, 0);
  useBuzzerOff();
}
/*
 * fungsi dasar yang digunakan untuk memutar robot sampai hitungan encoder tercapai
 */
void turnenc(bool dir, uint16_t speed, uint16_t count, uint16_t backbraketime)
{
  counterL = 0;
  counterR = 0;
  switch (dir)
  {
  case leftSide:
    while (counterR < count)
      kinematik(-speed, speed);
    break;
  case rightSide:
    while (counterL < count)
      kinematik(speed, -speed);
  }
  kinematik(0, 0);
  usebuzzeron();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < backbraketime)
  {
    switch (dir)
    {
    case leftSide:
      kinematik(speed, -speed);
      break;
    case rightSide:
      kinematik(-speed, speed);
    }
  }
  kinematik(0, 0);
  useBuzzerOff();
}
// **********************************************************************************************
//  dibawah ini merupakan fungsi-fungsi yang bisa digunakan oleh user
// **********************************************************************************************
/*
 * fungsi yang digunakan untuk membunyikan buzzer (dan/atau) led beberapa kali sesuai interval yang diinginkan
 */
void buzzerled(bool useBuzzer, bool useLED, uint8_t numberOfTimes, uint16_t interval, uint16_t customOffTime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncBuzzerLed.length + 15] = {0};
    sprintf(d, getStringFormat(FuncBuzzerLed).c_str(), nFunc, useBuzzer, useLED, numberOfTimes, interval, customOffTime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  bool s = true;
  for (uint16_t i = 0; i < (uint16_t)numberOfTimes * 2; i++)
  {
    if (s)
    {
      if (useBuzzer)
        digitalWrite(buzz, 1);
      if (useLED)
        digitalWrite(LED_BUILTIN, 1);
      delay(interval);
    }
    else
    {
      digitalWrite(buzz, 0);
      digitalWrite(LED_BUILTIN, 0);
      if ((i == numberOfTimes * 2 - 1) && numberOfTimes != 1)
        break;
      if (customOffTime != 0)
      {
        delay(customOffTime);
      }
      else
      {
        delay(interval);
      }
    }
    s = !s;
  }
  digitalWrite(buzz, 0);
  digitalWrite(LED_BUILTIN, 0);
}
/*
 * fungsi yang digunakan untuk mengatur pcTime, yaitu perbedaan waktu minimal sensor kanan dan kiri dalam mendeteksi garis untuk menentukan bahwa 
 * itu adalah percabangan, dan untuk mengatur apakah mau menggunakan buzzer atau tidak
 * 
 * iUB -> isUseBuzzer
 */
void start(bool b, uint16_t p)
{
  if (iMC)
    return;
  pcTime = p;
  iUB = b;
}
/*
 * fungsi untuk mereset startTime 
 */
void resettimer()
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncResetTimer.length + 2] = {0};
    sprintf(d, getStringFormat(FuncResetTimer).c_str(), nFunc);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  startTime = millis();
}
/*
 * fungsi untuk mengubah nilai pcTime
 */
void pctimer(uint16_t pc)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  pcTime = pc;
}
/*
 * fungsi untuk merubah warna garis yang digunakan
 */
void linecolor(bool color)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLineColor.length + 5] = {0};
    sprintf(d, getStringFormat(FuncLineColor).c_str(), nFunc, (color == black ? getStringFormat(InfoLineBlack).c_str() : getStringFormat(InfoLineWhite).c_str()));
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  if (color == black)
  {
    fsensor.print('B');
    rsensor.print('B');
  }
  else
  {
    fsensor.print('W');
    rsensor.print('W');
  }
}
/*
 * fungsi untuk merubah arah laju robot maju/atau mundur
 */
void sensor(bool dir)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  if (debugMode != no_debug)
  {
    char d[FuncSensor.length + 5] = {0};
    sprintf(d, getStringFormat(FuncSensor).c_str(), nFunc, (dir == ff ? getStringFormat(InfoFF).c_str() : getStringFormat(InfoBB).c_str()));
    log(d);
  }
  if (dir == ff)
  {
    iTF = true;
  }
  else
  {
    iTF = false;
  }
}
/*
 * fungsi untuk mengatur apakah menggunakan error(robot akan berhenti saat tidak mendeteksi garis) dan waktu maksimal robot tidak mendeteksi garis
 * jika robot sudah tidak mendeteksi garis lebih dari maxErrorTime maka robot akan berhenti dan membunyikan buzzer
 */
void error(bool useError, uint16_t time)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncError.length + 10] = {0};
    sprintf(d, getStringFormat(FuncError).c_str(), nFunc, useError, time);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  iUE = useError;
  maxErrorTime = time;
}
/*
 * fungsi untuk mengatur Kp dan Kd pid sensor garis
 */
void controller(float customKp, float customKd)
{
  Kp = constrain(customKp, 0.0, 100.0);
  Kd = constrain(customKd, 0.0, 100.0);

  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  if (debugMode != no_debug)
  {
    char p1[9] = {0};
    char p2[9] = {0};
    dtostrf(customKp, 2, 5, p1);
    dtostrf(customKd, 2, 5, p2);
    char d[FuncController.length + 10] = {0};
    sprintf(d, getStringFormat(FuncController).c_str(), nFunc, p1, p2);
    log(d);
  }
}
/*
 * fungsi untuk mengatur Kp, Ki dan Kd pid kontrol rpm motors
 */
void motorController(float customKp, float customKi, float customKd)
{
  KpMotor = customKp;
  KiMotor = customKi;
  KdMotor = customKd;

  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
}
/*
 * fungsi untuk menandai setpoin
 */
void step()
{
  mStep++;
  if (mStep < nStep)
    return;
  if (debugMode == by_step)
  {
    char d[InfoStep.length + 2] = {0};
    sprintf(d, getStringFormat(InfoStep).c_str(), mStep);
    log(d);
    while (digitalRead(btn4) || (unsigned long)millis() - btnTiming < delaybutton)
    {
    };
    btnTiming = millis();
    digitalWrite(buzz, 1);
    delay(delaybuzzer);
    digitalWrite(buzz, 0);
  }
  delay(stepDelayTime);
}
/*
 * fungsi untuk mengatur robot berhenti disetpoin berapa lama
 */
// todo: implementasi pengaturan nilai stepdelaytime agar robot berhenti di setpoin sesuai waktu yang telah diatur
void setstepdelay(uint16_t time)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  nFunc++;
}
/* 
 * fungsi untuk mengatur offset pwm motor saat maju agar seimbang kanan dan kiri
 */
void ffspeed(uint8_t l, uint8_t r)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncFfSpeed.length + 10] = {0};
    sprintf(d, getStringFormat(FuncFfSpeed).c_str(), nFunc, l, r);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  leftPwmOffsetFwd = l;
  rightPwmOffsetFwd = r;
}
/*
 * fungsi untuk mengatur offset pwm saat mundur agar seimbang kanan kiri
 */
void bbspeed(uint8_t l, uint8_t r)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncBbSpeed.length + 10] = {0};
    sprintf(d, getStringFormat(FuncBbSpeed).c_str(), nFunc, l, r);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  leftPwmOffsetBwd = l;
  rightPwmOffsetBwd = 1;
}
/*
 * fungsi untuk user memajukan / memundurkan robot selama waktu yang diatur tanpa membaca sensor garis
 */
void motor(int16_t leftSpeed, int16_t rightSpeed, uint16_t runtime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncMotor.length + 15] = {0};
    sprintf(d, getStringFormat(FuncMotor).c_str(), nFunc, leftSpeed, rightSpeed, runtime);
    log(d);
  };
  if (debugMode == by_func)
    waitKey4();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < runtime)
  {
    kinematik(leftSpeed, rightSpeed);
  }
}
/*
 * fungsi untuk membuat robot maju/mundur sejauh jarak cm yang diatur tanpa membaca sensor garis
 */
void motorcmavoider(int16_t speed, uint16_t cm, uint16_t backbraketime)
{
  if (iMC)
   return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  motorcmFunc(speed, cm, backbraketime, sideAvoidance);
}
/*
 * fungsi untuk membuat robot maju/mundur sejauh jarak cm yang diatur tanpa membaca sensor garis
 */
void motorcm(int16_t speed, uint16_t cm, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  motorcmFunc(speed, cm, backbraketime);
}

/*
 * fungsi untuk membuat robot maju/mundur dengan kecepatan rpm yang diatur selama waktu yang ditentukan tanpa membaca sensor garis
 */
void motorrpmFunc(uint16_t spmspeed, uint16_t runtime, uint16_t backbraketime, bool isAvoider)
{
  int16_t spdl = 0;
  int16_t spdr = 0;
  float iErrorL = 0;
  float iErrorR = 0;
  float dErrorL = 0;
  float dErrorR = 0;
  cleanSensor();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < runtime)
  {
    float rpmL = getRPML();
    float rpmR = getRPMR();

    float lError = spmspeed - rpmL;
    float rError = spmspeed - rpmR;

    float outL = KpMotor * lError + KiMotor * iErrorL + KdMotor * (lError - dErrorL);
    float outR = KpMotor * rError + KiMotor * iErrorR + KdMotor * (rError - dErrorR);

    iErrorL += lError;
    iErrorR += rError;

    iErrorL = constrain(iErrorL, -400, 400);
    iErrorR = constrain(iErrorR, -400, 400);

    dErrorL = lError;
    dErrorR = rError;

    spdl += outL;
    spdr += outR;
    spdl = constrain(spdl, -255, 255);
    spdr = constrain(spdr, -255, 255);

    if (isAvoider)
    {
      int16_t* spd = sidelineError();
      if (*(spd) == 404 && *(spd+1) == 404) // robot mendeteksi garis di tengah tengah sensor bisa jadi karena menabrak garis yang melintang di depannya
      {
        break;
      }
      if (*(spd) != 0)
        spdl = *(spd);
      if (*(spd+1) != 0)
        spdr = *(spd+1);
    }

    kinematik((iTF ? spdl : -spdl), (iTF ? spdr : -spdr));
    delay(10);
  }
  kinematik(0, 0);
}
/*
 * fungsi untuk membuat robot maju/mundur dengan kecepatan rpm yang diatur selama waktu yang ditentukan tanpa membaca sensor garis
 */
void motorrpm(uint16_t speed, uint16_t runtime, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  motorrpmFunc(speed, runtime, backbraketime, false);
}
/*
 * fungsi untuk membuat robot maju/mundur dengan kecepatan rpm yang diatur selama waktu yang ditentukan tanpa membaca sensor garis 
 * dan menghindari garis
 */
void motorrpmavoider(uint16_t speed, uint16_t runtime, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  motorrpmFunc(speed, runtime, backbraketime, true);
}
/*
 * fungsi untuk membuat robot maju/mundur dengan kecepatan yang diatur dan selama waktu yang telah ditentukan 
 * jika robot mendeteksi warna (menggunakan huskylens) maka robot akan berhenti
 * jika avoidactive diatur ke true, jika robot mendeteksi garis maka robot akan menghindari garis tersebut
 * colorId adalah id warna yang telah diatur di husky lens
 */
int8_t motorrpmdetectcolor(uint16_t spmspeed, uint16_t runtime, bool avoidactive, int8_t colorId)
{
  if (iMC)
    return 0;
  nFunc++;
  if (mStep < nStep)
    return 0;
  if (debugMode == by_func)
    waitKey4();
  int16_t spdl = 0;
  int16_t spdr = 0;
  float iErrorL = 0;
  float iErrorR = 0;
  float dErrorL = 0;
  float dErrorR = 0;
  PacketHusky packet;
  cleanSensor();
  int8_t colordetected = 0;
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < runtime)
  {
    packet = readHusky(bb, 'C');
    if (packet.id != 0)
    {
      if (colorId == 0)
      {
        colordetected = packet.id;
        break;
      }
      else
      {
        if (packet.id == colorId)
        {
          colordetected = packet.id;
          break;
        }
      }
    }

    float rpmL = getRPML();
    float rpmR = getRPMR();

    float lError = spmspeed - rpmL;
    float rError = spmspeed - rpmR;

    float outL = KpMotor * lError + KiMotor * iErrorL + KdMotor * (lError - dErrorL);
    float outR = KpMotor * rError + KiMotor * iErrorR + KdMotor * (rError - dErrorR);

    iErrorL += lError;
    iErrorR += rError;

    iErrorL = constrain(iErrorL, -400, 400);
    iErrorR = constrain(iErrorR, -400, 400);

    dErrorL = lError;
    dErrorR = rError;

    spdl += outL;
    spdr += outR;
    
    int16_t* s = sidelineError();
    if(s[0] != 0 && s[0] != 404) spdl = s[0];
    if(s[1] != 0 && s[1] != 404) spdr = s[1];
    if(s[0] == 404 && s[1] == 404) break;

    kinematik((iTF ? spdl : -spdl), (iTF ? spdr : -spdr));
    delay(5);
  }
  kinematik(0, 0);
  return colordetected;
}
/*
 * fungsi untuk membuat robot maju dengan kecepatan tertentu selama waktu yang telah ditentukan
 * jika robot mendeteksi qrcode maka robot akan berhenti walaupun runtime belum tercapai
 * jika avoidactive maka robot akan selalu menghidari jika ada garis 
 */
String motorrpmdetectqr(uint16_t spmspeed, uint16_t runtime, bool avoidactive)
{
  if (iMC)
    return "MODECOUNT";
  nFunc++;
  if (mStep < nStep)
    return "SKIPSTEP";
  if (debugMode == by_func)
    waitKey4();
  // not implemented
}
/*
 * robot akan bergerak maju/mundur sejauh jarak yang ditentukan dan menghindari garis
 */
void motorsideavoider(int16_t speed, uint16_t cm, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  motorcmFunc(speed, cm, backbraketime, sideAvoidance);
}
/*
 * robot akan berjalan lurus dengan kecepatan yang telah diatur sejauh jarak yang ditentukan
 * robot akan menjaga heading dengan threshold yang telah diatur
 */
void motorheading(int16_t speed, float headingRef, uint16_t cm, uint16_t threshold = 2)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  // menyamakan heading aktual dengan referensi
  usebuzzeron();
  while (abs(headingRef - heading) > threshold)
  {
    imusense();
    if (headingRef < heading)
    {
      kinematik(-speed, speed);
    }
    else
    {
      kinematik(speed, -speed);
    }
  }
  useBuzzerOff();
  // maju sejauh cm dan selalu koreksi heading
  counterR = 0;
  uint32_t step = 15 * cm;
  while (counterR < step)
  {
    imusense();
    int16_t error = headingRef - heading;
    int spdl = speed - error;
    int spdr = speed + error;
    kinematik(spdl, spdr);
  }
  kinematik(0, 0);
}
/*
 * robot menjalankan fungsi maze dengan mendeteksi percabangan, melakukan fungsi belok sesuai dir yang telah diatur
 * dan berjalan dengan kecepatan yang telah diatur
 */
void line(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLine.length + 20] = {0};
    sprintf(d, getStringFormat(FuncLine).c_str(), nFunc, method, dir, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  lineFunc(method, dir, speed, backbraketime);
}
void linet(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime, uint16_t actionTime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLineT.length + 30] = {0};
    sprintf(d, getStringFormat(FuncLineT).c_str(), nFunc, method, dir, speed, backbraketime, actionTime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  linetFunc(method, dir, speed, backbraketime, actionTime);
}
void timeline(uint8_t method, uint8_t dir, int16_t speed, int16_t backbraketime, uint16_t totalActionTime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncTimeline.length + 30] = {0};
    sprintf(d, getStringFormat(FuncTimeline).c_str(), nFunc, method, dir, speed, backbraketime, totalActionTime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  unsigned long startTime = millis();
  while ((unsigned long)millis() - startTime < totalActionTime)
  {
    detectorRun(method, dir, speed, backbraketime, -1);
  }
}
void linedelay(int16_t speed, uint16_t runtime, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLineDelay.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLineDelay).c_str(), nFunc, speed, runtime, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  linedelayFunc(speed, runtime, backbraketime);
}
void linefind(int16_t leftSpeed, int16_t rightSpeed, uint16_t timeToDisregardLine)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLineFind.length + 25] = {0};
    sprintf(d, getStringFormat(FuncLineFind).c_str(), nFunc, leftSpeed, rightSpeed, timeToDisregardLine);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  usebuzzeron();
  bool isFound = false;
  cleanSensor();
  unsigned long timeStart = millis();
  while (!isFound)
  {
    readSensor(iTF ? ff : bb);
    kinematik(leftSpeed, rightSpeed);
    for (uint8_t i = 0; i < 10; i++)
    {
      if (senData[i])
      {
        if ((unsigned long)millis() - timeStart > timeToDisregardLine)
        {
          isFound = true;
        }
      }
    }
  }
  kinematik(0, 0);
  useBuzzerOff();
}
void linedline(uint16_t runtime, uint16_t startSpeed, uint16_t method, uint16_t dir, uint16_t endSpeed, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLineDLine.length + 30] = {0};
    sprintf(d, getStringFormat(FuncLineDLine).c_str(), nFunc, runtime, startSpeed, method, dir, endSpeed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  linedelayFunc(startSpeed, runtime, 0);
  lineFunc(method, dir, endSpeed, backbraketime);
}
void linetline(uint16_t runtime, uint8_t startSpeed, uint8_t method, uint8_t dir, uint8_t endSpeed, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLineTLine.length + 30] = {0};
    sprintf(d, getStringFormat(FuncLineTLine).c_str(), nFunc, runtime, startSpeed, method, dir, endSpeed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  linetFunc(method, dir, startSpeed, 0, runtime);
  lineFunc(method, dir, endSpeed, backbraketime);
}
void slineFunc(uint16_t sensor, int16_t speed, int16_t backbraketime, int8_t dir)
{
  cleanSensor();
  bool line = false;
  while (!line)
  {
    readSensor(iTF ? ff : bb); 
    uint16_t l = 0; 
    senData2Bin(&l);
    if ((l & sensor) == sensor)
      line = true;
    switch (dir)
    {
      case centerSide: controllerRun(l, speed); break;
      case rightSide: controllerNoErrorRun(speed, false, rightSide); break;
      case leftSide: controllerNoErrorRun(speed, false, leftSide); break;
    }
  }
  usebuzzeron();
  // while (1)
  // {
  //   readSensor(iTF ? ff : bb);
  //   uint16_t l = 0;
  //   senData2Bin(&l);
  //   if ((l & sensor) == 0x00)
  //     break;
  //   switch (dir)
  //   {
  //     case centerSide: controllerRun(l, speed); break;
  //     case rightSide: controllerNoErrorRun(speed, false, rightSide); break;
  //     case leftSide: controllerNoErrorRun(speed, false, leftSide); break;
  //   }
  // }
  kinematik(0, 0);
  backbraketimeFunc(speed, backbraketime);
  useBuzzerOff();
}
void sline(uint16_t sensor, int16_t speed, int16_t backbraketime)
{
  if (iMC) // is mode count, agar saat menghitung jumlahset point, fungsi ini tidak berjalan
    return;
  nFunc++; // menandai jumlah fungsi yang telah dijalankan, 
  if (mStep < nStep) // jika setpoint yang dipilih lebih besar dari setpoint sekarang, maka lgsg keluar
    return;
  if (debugMode != no_debug)
  {
    char d[FuncSLine.length + 15] = {0};
    sprintf(d, getStringFormat(FuncSLine).c_str(), nFunc, sensor, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  slineFunc(sensor, speed, backbraketime, centerSide);
}
void slineright(uint16_t sensor, int16_t speed, int16_t backbraketime) 
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncSLine.length + 15] = {0};
    sprintf(d, getStringFormat(FuncSLine).c_str(), nFunc, sensor, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  slineFunc(sensor, speed, backbraketime, rightSide);
}
void slineleft(uint16_t sensor, int16_t speed, int16_t backbraketime) 
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncSLine.length + 15] = {0};
    sprintf(d, getStringFormat(FuncSLine).c_str(), nFunc, sensor, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  slineFunc(sensor, speed, backbraketime, leftSide);
}
void lostline(uint16_t lostLineTime, int16_t speed, uint16_t runtime, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLostLine.length + 30] = {0};
    sprintf(d, getStringFormat(FuncLostLine).c_str(), nFunc, lostLineTime, speed, runtime, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  bool flag = false;
  unsigned long detectTime = millis();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < runtime)
  {
    readSensor(iTF ? ff : bb);
    uint16_t l = 0;
    senData2Bin(&l);
    if (l != 0)
      flag = true;
    if (l == 0 && flag == false)
    {
      flag = true;
      detectTime = millis();
    }
    if (flag == true && (unsigned long)millis() - detectTime > lostLineTime)
      break;
    controllerRun(l, speed, false);
  }
  kinematik(0, 0);
  usebuzzeron();
  backbraketimeFunc(speed, backbraketime);
  useBuzzerOff();
}
/*
 * fungsi agar robot berjalan sesuai garis namun garis akan berada di sisi sebelah kiri
 * robot dengan kecepatan yang telah diatur dan selama waktu yang telah ditentukan
 * jika supermode aktif, maka robot akan sangat sensitif terhadap pertigaan 90 derajat
 */
void leftline(int16_t speed, uint16_t runtime, bool supermode)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  controllerNoErrorRun(speed, supermode, leftSide, runtime);
}
/*
 * fungsi agar robot mengikuti garis di sebelah kiri sensor dan berhenti saat mendeteksi qrcode
 * fungsi ini akan mengembalikan isi qrcode yang terdeteksi
 */
String leftlinedetectqr(int16_t speed, uint16_t runtime, bool supermode)
{
  if (iMC)
    return "COUNTMODE";
  nFunc++;
  if (mStep < nStep)
    return "SKIPSTEP";
  if (debugMode == by_func)
    waitKey4();
  cleanSensor();
  PacketRaspi packet;
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < runtime)
  {
    packet = readRaspi(bb, 1);
    if (packet.data != "")
      break;

    int16_t pwm = 0;
    int16_t error = leftlineError(supermode);
    pwm = Kp * error + Kd * (error - dError);
    pwm = constrain(pwm, -255, 255);
    dError = error;
    int16_t spdl = speed + pwm;
    int16_t spdr = speed - pwm;
    kinematik((iTF ? spdl : -spdl), (iTF ? spdr : -spdr));
  }
  kinematik(0, 0);
  return packet.data;
}
/*
 * fungsi agar robot berjalan sesuai garis namun garis akan berada di sisi sebelah kanan
 * robot dengan kecepatan yang telah diatur dan selama waktu yang telah ditentukan
 * jika supermode aktif, maka robot akan sangat sensitif terhadap pertigaan 90 derajat
 */
void rightline(int16_t speed, uint16_t runtime, bool supermode)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  controllerNoErrorRun(speed, supermode, rightSide, runtime);
}
/*
 * robot akan berputar kekiri sampai sensor 2 mendeteksi garis
 */
void left(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 1, speed, backbraketime);
}
void left1(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft1.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft1).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 0, speed, backbraketime);
}
void left2(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft2.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft2).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 1, speed, backbraketime);
}
void left3(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft3.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft3).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 2, speed, backbraketime);
}
void left4(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft4.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft4).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 3, speed, backbraketime);
}
void left5(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft5.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft5).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 4, speed, backbraketime);
}
void left6(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft6.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft6).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 5, speed, backbraketime);
}
void left7(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncLeft7.length + 15] = {0};
    sprintf(d, getStringFormat(FuncLeft7).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(leftSide, 6, speed, backbraketime);
}
/*
 * robot akan berputar ke kiri sampai hitungan encoder yang diatur tercapai dengan kecepatan yang telah diatur
 */
void leftenc(int16_t speed, uint16_t count, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  turnenc(leftSide, speed, count, backbraketime);
}
void right(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 6, speed, backbraketime);
}
void right10(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight10.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight10).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 9, speed, backbraketime);
}
void right9(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight9.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight9).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 8, speed, backbraketime);
}
void right8(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight8.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight8).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 7, speed, backbraketime);
}
void right7(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight7.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight7).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 6, speed, backbraketime);
}
void right6(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight6.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight6).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 5, speed, backbraketime);
}
void right5(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight5.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight5).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 4, speed, backbraketime);
}
void right4(int16_t speed, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncRight4.length + 15] = {0};
    sprintf(d, getStringFormat(FuncRight4).c_str(), nFunc, speed, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  turn(rightSide, 3, speed, backbraketime);
}
/*
 * robot akan berputar ke kanan sesuai dengan jumlah encoder yang diinginkan dengan kecepatan yang telah diatur
 */
void rightenc(int16_t speed, uint16_t count, uint16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  turnenc(rightSide, speed, count, backbraketime);
}
/*
 * robot akan berbelok sesuai sudut yang diinginkan
 */
// todo: implementasi kode ini dengan gabungan pembacaan imusense()
void turnangle(int16_t angle)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
}
void exline(int16_t leftMotorSpeed, int16_t rightMotorSpeed, uint8_t sensor, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncExline.length + 30] = {0};
    sprintf(d, getStringFormat(FuncExline).c_str(), nFunc, leftMotorSpeed, rightMotorSpeed, sensor, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  bool lineDetected = false;
  while (!lineDetected)
  {
    readSensor(iTF ? ff : bb);
    uint16_t l;
    senData2Bin(&l);
    if ((l & sensor) == sensor)
      lineDetected = true;
    kinematik((iTF ? leftMotorSpeed : -leftMotorSpeed), (iTF ? rightMotorSpeed : -rightMotorSpeed));
  }
  while (lineDetected)
  {
    readSensor(iTF ? ff : bb);
    uint16_t l = 0;
    senData2Bin(&l);
    if ((l & sensor) == 0x00)
      lineDetected = false;
    kinematik((iTF ? leftMotorSpeed : -leftMotorSpeed), (iTF ? rightMotorSpeed : -rightMotorSpeed));
  }
  usebuzzeron();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < backbraketime)
  {
    kinematik((iTF ? leftMotorSpeed : -leftMotorSpeed), (iTF ? rightMotorSpeed : -rightMotorSpeed));
  }
  kinematik(0, 0);
  useBuzzerOff();
}
void exturn(int16_t leftMotorSpeed, int16_t rightMotorSpeed, uint8_t sensor, int16_t backbraketime)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode != no_debug)
  {
    char d[FuncExturn.length + 20] = {0};
    sprintf(d, getStringFormat(FuncExturn).c_str(), leftMotorSpeed, rightMotorSpeed, sensor, backbraketime);
    log(d);
  }
  if (debugMode == by_func)
    waitKey4();
  bool lineDetected = false;
  while (!lineDetected)
  {
    readSensor(iTF ? ff : bb);
    uint16_t l = 0;
    senData2Bin(&l);
    if ((l & sensor) == sensor)
      lineDetected = true;
    kinematik((iTF ? leftMotorSpeed : -leftMotorSpeed), (iTF ? rightMotorSpeed : -rightMotorSpeed));
  }
  kinematik(0, 0);
  usebuzzeron();
  unsigned long timeStart = millis();
  while ((unsigned long)millis() - timeStart < backbraketime)
  {
    kinematik((iTF ? leftMotorSpeed : -leftMotorSpeed), (iTF ? rightMotorSpeed : -rightMotorSpeed));
  }
  kinematik(0, 0);
  useBuzzerOff();
}
/*
 * fungsi untuk menggerakkan servo yang berada di pin yang telah diatur dengan sudut yang diinginkan
 */
void servo(uint8_t pin, uint16_t deg)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  uint16_t pulse = deg / 180.0 * 450 + 150;
  srv.setPWM(pin, 0, pulse);
}
/*
 * fungsi bawaan untuk melakukan penjepitan benda dan mengangkat keatas
 */
void pickup(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(6, 120); // sesuaikan
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 * fungsi bawaan untuk melakukan pembukaan gripper 
 */
void placeup(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(7, 160); // sesuaikan
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 *
 */
void pickdn(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(6, 60); // sesuaikan
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 *
 */
void placedn(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(7, 90); // sesuaikan
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 *
 */
void take(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(6, 60); // sesuaikan
  delay(timedelay);
  servo(7, 90);
  delay(timedelay);
  servo(6, 120);
  delay(timedelay);
  servo(7, 160);
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 *
 */
void put(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(7, 90); // sesuaikan
  delay(timedelay);
  servo(6, 60);
  delay(timedelay);
  servo(7, 160);
  delay(timedelay);
  servo(6, 120);
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 * fungsi bawaan untuk mengatur kamera menghadap ke kanan
 */
void camright(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(5, 35); // sesuaikan yaw
  servo(6, 120); // sesuaikan pitch
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 * fungsi bawaan untuk mengatur kamera menghadap ke depan
 */
void camfront(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(5, 130); // sesuaikan
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 * fungsi bawaan untuk mengatur kamera menghadap ke kiri
 */
void camleft(uint16_t timedelay)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  servo(5, 210); // sesuaikan
  usebuzzeron();
  delay(timedelay);
  useBuzzerOff();
}
/*
 * fungsi untuk mengambil warna yang dideteksi
 * which sensor diisi dengan (ff/bb) sesuai dengan disensor mana huskylens disambungkan
 */
uint8_t camdetectcolor(bool ws)
{
  if (iMC)
    return 0;
  nFunc++;
  if (mStep < nStep)
    return 0;
  if (debugMode == by_func)
    waitKey4();
  PacketHusky packet;
  packet = readHusky(ws, 'C');
  return packet.id;
}
/*
 * fungsi untuk mengambil nilai qrcode yang dideteksi menggunakan modul gm66
 */
String gm66detectqr(bool ws, int8_t t)
{
  if (iMC)
    return "MC";
  nFunc++;
  if (mStep < nStep)
    return "SS";
  if (debugMode == by_func)
    waitKey4();
  String qrcode = "";
  bool fail = false;
  HardwareSerial *os = (ws == ff ? &fsensor : &rsensor);
  cleanSensor();
  os->write('G');
  while(!os->available() && t > 0)
  {
    if(--t <= 0) fail = true;
    delay(1000);
  }
  os->write('F');
  if(fail == true)
  {
    return "";
  }
  else
  {
    qrcode = os->readStringUntil('\r');
    cleanSensor();
    os->write('K');
    return qrcode;
  }
}
/*
 * fungsi untuk mengambil nilai qrcode yang dideteksi menggunakan modul raspberrypi
 */
String raspidetectqr(bool ws, int8_t t)
{
  if (iMC)
    return "MC";
  nFunc++;
  if (mStep < nStep)
    return "SS";
  if (debugMode == by_func)
    waitKey4();
  PacketRaspi packet;
  while (packet.data == "" && t > 0)
  {
    t--;
    packet = readRaspi(ws);
    showonlcd((String)t + ". " + packet.data);
    if (packet.data == "") delay(1000);
  }
  return packet.data;
}
/*
 * fungsi bawaan jika anda ingin data anda (apapun itu) tampil di lcd
 */
void showonlcd(String data)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  if (debugMode == by_func)
    waitKey4();
  display.clear();
  display.setCursorXY(0, 0);
  display.print(data);
  display.update();
}
/*
 * fungsi pengganti fungsi delay agar robot tidak seperti hang saat boot up
 */
void delay_maze(uint32_t delayMilli)
{
  if (iMC)
    return;
  nFunc++;
  if (mStep < nStep)
    return;
  delay(delayMilli);
}
// ********************************************************************************
// ****************************************************************** main function
void initMazeRobot()
{
  // serial sensor init
  Serial.begin(115200);
  fsensor.begin(115200);
  rsensor.begin(115200);
  // i2c communication init
  Wire.begin();
  delay(10);
  // servo driver init
  srv.begin();
  srv.setPWMFreq(60);
  // pin io mode init
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);
  pinMode(pwm3, OUTPUT);
  pinMode(pwm4, OUTPUT);
  pinMode(buzz, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // default pin output value
  digitalWrite(pwm1, 0);
  digitalWrite(pwm2, 0);
  digitalWrite(pwm3, 0);
  digitalWrite(pwm4, 0);
  digitalWrite(buzz, 0);
  digitalWrite(LED_BUILTIN, 0);
  // button init
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  pinMode(btn3, INPUT_PULLUP);
  pinMode(btn4, INPUT_PULLUP);
  // external interrupt init
  pinMode(PA0, INPUT);
  pinMode(PA1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PA0), pulseCountL, RISING);
  attachInterrupt(digitalPinToInterrupt(PA1), pulseCountR, RISING);
  // display init
  display.init();
  display.clear();
  display.setScale(1);
  display.invertDisplay(false);
  // custom maze robot init
  initMazeRobot();
  // first setup
  getLastMode();
  countSetPoint();
  buzzer(3, 50, 100);
  // main menu pemilihan mode, setpoin, dan debugging
  unsigned long lcdTiming = millis();
  btnTiming = millis();
  while (1)
  {
    // jika button1 ditekan, maka akan mengganti mode dari 1-3
    if (isBtn1() && (unsigned long)millis() - btnTiming > delaybutton)
    {
      digitalWrite(buzz, 1);
      btnTiming = millis();
      if (++runMode > 3)
        runMode = 1;
      countSetPoint();
      delay(delaybuzzer);
      digitalWrite(buzz, 0);
    }
    // jika button2 ditekan, maka akan mengganti setpoin sesuai dengan jumlah
    // setpoin yang telah ditambahkan oleh user di masing-masing mode
    if (isBtn2() && (unsigned long)millis() - btnTiming > delaybutton)
    {
      digitalWrite(buzz, 1);
      btnTiming = millis();
      if (++nStep > mStep)
        nStep = 0;
      delay(delaybuzzer);
      digitalWrite(buzz, 0);
    }
    // jika button3 ditekan maka akan mengganti debugging mode antara
    // no_debug, by_func dan by_step
    if (isBtn3() && (unsigned long)millis() - btnTiming > delaybutton)
    {
      digitalWrite(buzz, 1);
      btnTiming = millis();
      if (++debugMode > 2)
        debugMode = 0;
      delay(delaybuzzer);
      digitalWrite(buzz, 0);
    }
    // jika button4 ditekan maka user akan menjalankan robot sesuai mode, setpoin dan debugging yang telah diatur
    if (isBtn4() && (unsigned long)millis() - btnTiming > delaybutton)
    {
      digitalWrite(buzz, 1);
      btnTiming = millis();
      delay(delaybuzzer);
      digitalWrite(buzz, 0);
      break;
    }
    // timing lcd agar lcd tidak error
    if ((unsigned long)millis() - lcdTiming > delaybuzzer)
    {
      displayMenu();
      lcdTiming = millis();
    }
  }
  // run confirmation
  display.clear();
  display.home();
  display.print(getStringFormat(MenuMode));
  display.println(runMode);
  display.print(getStringFormat(MenuStep));
  display.println(nStep);
  display.print(getStringFormat(MenuDebug));
  switch (debugMode)
  {
  case no_debug:
    display.println(getStringFormat(InfoNoDebug));
    break;
  case by_func:
    display.println(getStringFormat(InfoByFunc));
    break;
  case by_step:
    display.println(getStringFormat(InfoByStep));
    break;
  default:
    debugMode = 0;
    break;
  }
  display.println();
  display.println();
  display.println(getStringFormat(MenuNormal));
  display.update();
  while (1)
  {
    if (isBtn4() && (unsigned long)millis() - btnTiming > delaybutton)
      break;
  }
  // start running
  log(F("run"));
  // save last config
  setLastMode();
  // variable initialization
  nFunc = 0;
  mStep = -1;
  startTime = millis();
  // run mode
  switch (runMode)
  {
  case 1:
    mode1();
    break;
  case 2:
    mode2();
    break;
  case 3:
    mode3();
    break;
  }
  // show total time
  char tf[15] = {0};
  dtostrf((millis() - startTime) / 1000.0, 10, 3, tf);
  char d[InfoYourTime.length + 10] = {0};
  sprintf(d, getStringFormat(InfoYourTime).c_str(), tf);
  log(d);
  // finish sound
  buzzer(10, 50, 100);
}

void loop()
{
  
}