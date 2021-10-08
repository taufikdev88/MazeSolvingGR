/*
 *  Modul sensor berkomunikasi dengan kamera huskylens dan berkomunikasi dengan modul main board
 *  Mainboard ke sensor:
 *  1. A -> meminta data sensor
 *  2. B -> set line color ke hitam (black)
 *  3. W -> set line color ke putih (white)
 *  4. T -> meminta jumlah sensor
 *  5. Q -> meminta huskylens baca qrcode
 *  6. C -> meminta huskylens baca warna
 *  7. L -> meminta huskylens baca garis
 * 
 *  Sensor ke Mainboard:
 *  1. data sensor H0000000000T
 *  2. tidak mengirim balasan untuk perintah B
 *  3. tidak mengirim balasan untuk perintah W
 *  4. H10T
 *  5. HB255,255,255,255,255,1T jika mendapat data block
 *     HA255,255,255,255,255,1T jika mendapat data arrow
 *     N -> Object Warna/QrCode/Garis/Wajah ada tetapi tidak ada yang dikenali
 *     F0 -> Huskylens tidak ready
 *     F1 -> Koneksi dengan husky lens terputus
 *     F2 -> Tidak ada pembelajaran sama sekali
 *     F3 -> Tidak ada objek yang di deteksi
 * 
 */

#include <Wire.h>
#include "HUSKYLENS.h"

#define s0 PB1
#define s1 PB0
#define s2 PA7
#define s3 PA6
#define s4 PA5
#define s5 PA4
#define s6 PA3
#define s7 PA2
#define s8 PA1
#define s9 PA0

#define b0 PB4
#define b1 PB5

#define l0 PA8

const uint8_t sPin[10] = {s0, s1, s2, s3, s4, s5, s6, s7, s8, s9};
const int sensorTotal = (sizeof(sPin) / sizeof(sPin[0]));
uint16_t thSensor[sensorTotal] = {0};
bool isBlackLine = true;

unsigned long timing = 0;
bool flag = false;
bool huskyready = false;
char huskymode = 'L'; // Q,C,L

HUSKYLENS huskylens;

/*
 * Fungsi mengirim hasil dari Husky Lens ke board utama
 */
void printResult(HUSKYLENSResult result)
{
  if (result.command == COMMAND_RETURN_BLOCK)
  {
    Serial3.print('H'); // head of packet
    Serial3.print('B');
    Serial3.print(result.xCenter);
    Serial3.print(',');
    Serial3.print(result.yCenter);
    Serial3.print(',');
    Serial3.print(result.width);
    Serial3.print(',');
    Serial3.print(result.height);
    Serial3.print(',');
    Serial3.print(result.ID);
    Serial3.print('T'); // tail of packet
  }
  else if (result.command == COMMAND_RETURN_ARROW)
  {
    Serial3.print('H'); // head of packet
    Serial3.print('A');
    Serial3.print(result.xOrigin);
    Serial3.print(',');
    Serial3.print(result.yOrigin);
    Serial3.print(',');
    Serial3.print(result.xTarget);
    Serial3.print(',');
    Serial3.print(result.yTarget);
    Serial3.print(',');
    Serial3.print(result.ID);
    Serial3.print('T'); // tail of packet
  }
  else
  {
    Serial3.print('N');
  }
}

/*
 * Fungsi ini dipanggil saat mainboard meminta membaca huskylens
 */
void readHuskyLens()
{
  if (huskyready)
  {
    if (!huskylens.request())
      Serial3.print(F("F1"));
    else if (!huskylens.isLearned())
      Serial3.print(F("F2"));
    else if (!huskylens.available())
      Serial3.print(F("F3"));
    else
    {
      HUSKYLENSResult result = huskylens.read();
      printResult(result);
    }
  }
  else
  {
    if (huskylens.begin(Serial1))
    {
      huskyready = true;
      Serial3.print('N');
    }
    else
    {
      huskyready = false;
      Serial3.print('F');
    }
  }
}

/*
 * Fungsi membaca qrcode dari GM66
 */
void readGM66(){
  unsigned long timewait = millis();
  bool fail = false;
  while(!Serial1.available()){
    if((unsigned long) millis()-timewait >= 1000){
      fail = true;
      break;
    }
  }
  if(fail){
    Serial3.print(';');
    return;
  }
  Serial3.println(Serial1.readStringUntil('\n'));
}

/*
 * Fungsi membaca tombol dengan sampling
 */
bool readBtn(uint8_t pin)
{
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    if (digitalRead(pin) == 0)
      cnt++;
    delayMicroseconds(10);
  }
  if (cnt > 4)
    return true;
  return false;
}

/*
 * Fungsi mengambil nilai analog dari sebuah sensor, dengan fungsi sampling agar
 * nilainya lebih akurat
 */
uint16_t getSensorValue(uint8_t pin, bool sampling = true)
{
  uint32_t sensorValue = 0;
  if (sampling)
  {
    uint8_t i = 0;
    for (i = 1; i <= 10; i++)
    {
      sensorValue += analogRead(pin);
      delayMicroseconds(10);
    }
    sensorValue = sensorValue / 10;
  }
  else
  {
    sensorValue = analogRead(pin);
    delayMicroseconds(10);
  }
  return sensorValue;
}

/*
 * Fungsi simpan byte ke eeprom external
 */
void writeByteEEPROM(uint16_t address, byte data)
{
  Wire.beginTransmission(0x50);
  Wire.write((uint8_t)(address >> 8));
  Wire.write((uint8_t)(address & 0xFF));
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

/*
 * Fungsi ambil byte dari eeprom
 */
byte readByteEEPROM(uint16_t address)
{
  byte data = 0xFF;
  Wire.beginTransmission(0x50);
  Wire.write((uint8_t)(address >> 8));
  Wire.write((uint8_t)(address & 0xFF));
  Wire.endTransmission();
  Wire.requestFrom(0x50, 1);
  if (Wire.available())
    data = Wire.read();
  return data;
}

/*
 * Fungsi menyimpan nilai sebuah sensor ke eeprom
 */
void saveValue(int8_t s, uint16_t v)
{
  uint8_t high = v >> 8;
  uint8_t low = v & 0xFF;
  writeByteEEPROM(s * 2, high);
  writeByteEEPROM(s * 2 + 1, low);
}

/*
 * Mengambil nilai sebuah sensor dari eeprom
 */
uint16_t readValue(int8_t s)
{
  uint16_t data = 0x0000;
  data = readByteEEPROM(s * 2);
  data = data << 8;
  data = data | readByteEEPROM(s * 2 + 1);
  return data;
}

/*
 * Kalibrasi sensor selama 10 detik untuk mengambil nilai minimal maksimal
 */
void calibrateSensor()
{
  // siapkan variabel
  uint16_t value[sensorTotal][2] = {0};
  uint16_t realValue = 0;
  // set nilai variabel awal ke tengah tengah
  for (uint8_t i = 0; i < sensorTotal; i++)
  {
    value[i][0] = 1024;
    value[i][1] = 0;
  }
  // jalankan kalibrasi selama 10 detik
  unsigned long timestart = millis();
  while ((unsigned long)millis() - timestart <= 10000)
  {
    digitalWrite(l0, (((unsigned long)millis() - timestart) / 500) % 2);
    for (uint8_t i = 0; i < sensorTotal; i++)
    {
      realValue = getSensorValue(sPin[i]);
      if (realValue < value[i][0])
        value[i][0] = realValue;
      if (realValue > value[i][1])
        value[i][1] = realValue;
      thSensor[i] = (value[i][0] + value[i][1]) / 2;
    }
  }
  digitalWrite(l0, 0);
}

//******************************** SETUP ********************************
void setup()
{
  Serial3.begin(115200);
  Serial1.begin(115200);
  Wire.begin();

  for (uint8_t i = 0; i < sensorTotal; i++)
  {
    pinMode(sPin[i], INPUT);
    thSensor[i] = readValue(i);
  }

  pinMode(b0, INPUT_PULLUP);
  pinMode(b1, INPUT_PULLUP);
  pinMode(l0, OUTPUT);
  digitalWrite(l0, 0);

  if (!huskylens.begin(Serial1))
  {
    huskyready = false;
  }
  else
  {
    huskyready = true;
    huskymode = 'L';
    huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
  }
}
//******************************** LOOP  ********************************
void loop()
{
  if (Serial3.available())
  {
    char cIn = Serial3.read();

    switch (cIn)
    {
    case 'A':
      Serial3.print(F("H"));
      for (uint8_t i = 0; i < sensorTotal; i++)
      {
        bool biggerThanThreshold = (getSensorValue(sPin[i]) < thSensor[i]);
        if (isBlackLine)
          biggerThanThreshold = !biggerThanThreshold;
        Serial3.print(biggerThanThreshold);
      }
      Serial3.print('T');
      break;
    case 'B':
      isBlackLine = true;
      break;
    case 'W':
      isBlackLine = false;
      break;
    case 'T':
      Serial3.print('H');
      Serial3.print(sensorTotal);
      Serial3.print('T');
      break;
    case 'H':
      readHuskyLens();
      break;
    case 'L':
      if (huskymode != 'L' && huskyready)
      {
        huskymode = 'L';
        huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
      }
      readHuskyLens();
      break;
    case 'Q':
      if (huskymode != 'Q' && huskyready)
      {
        huskymode = 'Q';
        huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
      }
      readHuskyLens();
      break;
    case 'C':
      if (huskymode != 'C' && huskyready)
      {
        huskymode = 'C';
        huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
      }
      readHuskyLens();
      break;
    case 'G':
      readGM66();
      break;
    }
  }

  if (flag == true)
  {
    if (readBtn(b0))
    {
      flag = false;
      digitalWrite(l0, 0);
    }
    if (readBtn(b1))
    {
      flag = false;
      calibrateSensor();
      for (uint8_t i = 0; i < sensorTotal; i++)
      {
        saveValue(i, thSensor[i]);
      }
    }
  }
  else
  {
    if (readBtn(b0))
    {
      timing = millis();
      digitalWrite(l0, 1);
      while (readBtn(b0))
        ;
      if ((unsigned long)millis() - timing > 1000)
      {
        flag = true;
      }
      else
      {
        digitalWrite(l0, 0);
      }
      delay(500);
      Serial3.flush();
    }
  }
}