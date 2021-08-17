void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);
}

char command = 'L';
void loop() {
  if(Serial.available() > 0){
    char d = (char) Serial.read();
    if(d != 'C' && d != 'L' && d != 'Q') return;
    command = d; 
  }
  
  Serial2.write(command);
  bool errorFlag = false;
  uint16_t xa = 0, xb = 0, ya = 0, yb = 0;
  uint8_t id = 0, index = 0, i = 0;
  String data = "";
  
  while(Serial2.available()){
    char c = (char) Serial2.read();
    
    if(i == 0){
      errorFlag = false;
      if(c == 'H'){
//        Serial.println(F("Mendapatkan data"));
        Serial.print(c);
      } else
      if(c == 'N'){
//        Serial.println(F("Tidak dikenali"));
        Serial.println(c);
        i = 0;
      } else
      if(c == 'F'){
//        Serial.println(F("Error Flag aktif"));
        Serial.print(c);
        errorFlag = true;
      } else {
//        Serial.println(F("Data error"));
        Serial.println(c);
        i = 0;
      }
    } else
    if(i == 1){
      if(errorFlag){
        Serial.println(c);
//        if(c == '0') Serial.println(F("Husky lens tidak ready"));
//        else if(c == '1') Serial.println(F("Koneksi husky lens terputus"));
//        else if(c == '2') Serial.println(F("Perlu learning"));
//        else if(c == '3') Serial.println(F("Tidak ada objek terdeteksi"));
        
        i = 0;
      } else {
//        if(c == 'A') Serial.println(F("Data arrow didapat"));
//        else if(c == 'B') Serial.println(F("Data block didapat"));
        Serial.print(c);
      }
      index = 0;
    } else {
      if(c == ',' || c == 'T'){
        Serial.print(c);
        if(index == 0) xa = data.toInt();
        else if(index == 1) ya = data.toInt();
        else if(index == 2) xb = data.toInt();
        else if(index == 3) yb = data.toInt();
        else if(index == 4) {
          id = data.toInt();
          Serial.println((String) "   xa:" + xa + ", ya:" + ya + ", xb:" + xb + ", yb:" + yb + ", id:" + id);
          i = 0;
        }
        index++;
        data = "";
      } else 
      if(isDigit(c)){
        Serial.print(c);
        data += c;
      } else {
        Serial.println(F(" CORRUPT"));
        xa = xb = ya = yb = id = 0;
        data = "";
      }
    }
    i++;
  }

  delay(100);
}