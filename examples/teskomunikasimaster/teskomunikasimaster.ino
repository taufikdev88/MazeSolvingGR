void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);
}

void loop(){
  if(Serial.available() > 0){
    char a = (char) Serial.read();
    Serial1.write(a);
    Serial2.write(a);
  }

  if(Serial1.available() > 0){
    Serial.write(Serial1.read());
  }

  if(Serial2.available() > 0){
    Serial.write(Serial2.read());
  }
}