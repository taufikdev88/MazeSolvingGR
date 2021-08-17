/*
 * Program huskylens example untuk board sensor
 * kondisi huskylens menancap di Serial3 (serial yg digunakan untuk komunikasi dengan mainboard)
 */
#include "HUSKYLENS.h"

HUSKYLENS huskylens;

void printResult(HUSKYLENSResult result);
bool huskyread = false;

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);
    
    pinMode(PB4, INPUT_PULLUP);
    if(!huskylens.begin(Serial3)){
      huskyread = false;
    } else {
      huskyread = true;
    }
}

void loop() {
  if(Serial.available() > 0){
    char C = (char) Serial.read();
    if(C == 'C'){
      huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);
    } else
    if(C == 'L'){
      huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING);
    } else
    if(C == 'Q'){
      huskylens.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
    } else
    if(C == 'O'){
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_RECOGNITION);
    } else 
    if(C == 'T'){
      huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
    } else
    if(C == 'F'){
      huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION);
    }
  }
  
  if(huskyread){
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        HUSKYLENSResult result = huskylens.read();
        printResult(result);
    } 
  } else {
    Serial.println(F("Husky not available"));
    if(digitalRead(PB4) == 0){
      while(digitalRead(PB4)){};
      Serial.println(F("Try to reinitialize"));
      if(!huskylens.begin(Serial3)){
        Serial.println(F("cannot initialize"));
        huskyread = false;
      } else {
        Serial.println(F("success initialize"));
        huskyread = true;
      }
    }
  }
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    } else if (result.command == COMMAND_RETURN_ARROW){
        Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    } else{
        Serial.println("Object unknown!");
    }
}