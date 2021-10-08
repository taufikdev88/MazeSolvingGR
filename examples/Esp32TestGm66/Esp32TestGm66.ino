/*******************************
 * Program untuk diupload ke esp32 yang tersambung dengan sensor gm66
 * *******************************/
#include <SoftwareSerial.h>
SoftwareSerial testSerial;

void setup() {
  Serial.begin(115200);
  testSerial.begin(9600, SWSERIAL_8N1, 4, 2, false, 256);
}

void loop() {
  if(Serial.available()>0){
    Serial.write(Serial.read());
  }
  if(testSerial.available()>0){
    String data = "";
    while(testSerial.available() > 0){
      delay(10);
      data += (char) testSerial.read();
      yield();
    }
    Serial.println(data);
  }
}