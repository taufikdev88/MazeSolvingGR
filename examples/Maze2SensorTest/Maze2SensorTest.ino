/*******
 * Program untuk tes command dan membaca respon dari maze_sensor
 * *******/

#define rsensor Serial2
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rsensor.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    char c = (char) Serial.read();
    if(c != '\r' && c != '\n'){
      rsensor.print(c);
    }
  }
  if(rsensor.available()){
    Serial.write(rsensor.read());
  }
}