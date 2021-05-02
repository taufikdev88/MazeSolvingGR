#include "MazeCalibration.cpp"

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
// fungsi untuk tampilan saat user memasuki mode setting tampilan 1
void displayConfig1(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(getText(1));
  display.println(getText(2));
  display.println(getText(3));
  display.println(getText(4));
  display.display();
}
// fungsi untuk tampilan saat user memasuki mode setting tampilan 2
void displayConfig2(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(getText(5));
  display.println(getText(6));
  display.println(getText(7));
  display.println(getText(8));
  display.display();
}
// fungsi untuk mengetes/melihat nilai logika sensor saat dalam mode setting
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
// fungsi untuk menyuruh sensor untuk menyimpan nilai yang sudah sesuai ke eeprom masing masing
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
// fungsi untuk menyuruh sensor melakukan prosedur kalibrasi
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
