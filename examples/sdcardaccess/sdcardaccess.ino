#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>

File root;
bool isSuccess = false;

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

uint8_t iconTemp[1024] = {0};

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card on port: ");
  Serial.print(PA4);
  Serial.print("...");

  if (!SD.begin(PA4)) {
    Serial.println("initialization failed!");
    isSuccess = false;
    return;
  }
  isSuccess = true;
  Serial.println("initialization done.");

  root = SD.open("/");

  printDirectory(root, 0);
  Serial.println("done!");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.display();
}

enum menu {
  list,
  read,
  remo,
  write,
  show,
  process,
  none
} Menu;

String filename = "";
void loop() {
  if(isSuccess){
    if(Serial.available() > 0){
      char C = Serial.read();
  
      switch(C){
        case 'L':
        Menu = list;
        filename = "";
        break;
        case 'R':
        Menu = read;
        filename = "";
        break;
        case 'D':
        Menu = remo;
        filename = "";
        break;
        case 'W':
        Menu = write;
        filename = "";
        break;
        case 'S':
        Menu = show;
        break;
        case 'P':
        Menu = process;
        break;
        case '\n':

        switch(Menu){
          case list:
          Serial.print("listing");
          break;
          case read:
          Serial.print("reading ");
          break;
          case remo:
          Serial.print("deleting ");
          break;
          case write:
          Serial.print("writing ");
          break;
          case show:
          Serial.print("show to OLED ");
          break;
        }
        Serial.println(filename);
        
        switch(Menu){
          case list:
          root = SD.open("/");
          printDirectory(root, 0);
          root.close();
          break;
          case read:
          root = SD.open(filename);
          if(root){
            while(root.available()){
              Serial.write(root.read());
            }
            Serial.println();
            Serial.println("reading finish");
            root.close();
          } else {
            Serial.println((String) "cannot find file with name: " + filename);
          }
          break;
          case remo:
          if(SD.exists(filename)){
            SD.remove(filename);
            Serial.println("removing finish");
          } else {
            Serial.println("could not find file specified");
          }
          break;
          case write:
          root = SD.open(filename, FILE_WRITE);
          if(root){
            Serial.println("enter a text to write! end with: \"|\" character ");
            while(true){
              if(Serial.available() > 0){
                char D = Serial.read();
                
                if(D != '|'){
                  Serial.print(D);
                  root.print(D);
                } else {
                  root.close();
                  Serial.println("\nwriting finish");
                  break;
                }
              }
            }
          } else {
            Serial.println("can't open file to writing");
          }
          break;
          case show:
          root = SD.open(filename);
          if(root){
            filename = ""; // reuse the global variable to store text
            uint16_t i = 0;
            while(root.available()){
              char D = root.read();
              if(isDigit(D)){
                filename += D;
              } else if(D == ',') {
                iconTemp[i] = filename.toInt();
                filename = "";
                if(++i == 1024){
                  display.clearDisplay();
                  display.setCursor(0,0);
                  display.drawBitmap(0,0,iconTemp,128,64,WHITE);
                  display.display();
                }
              } else {
                Serial.println("find unidentified characted");
                Serial.println("canceling");
                break;
              }
            }
            Serial.println("reading finish");
            Serial.print("found ");
            Serial.print(i);
            Serial.println(" character length");
            Serial.println("note: Only show 1024 file length to OLED");
          } else {
            Serial.println("cannot open file specified");
          }
          break;
        }
        Menu = none;
        filename = "";
        break;
        case process:
        root = SD.open(filename);
        if(root){
          int8_t function;
          int8_t p1;
          int8_t p2;
          int8_t p3;
          while(root.available()){
            
          }
        } else {
          Serial.println("cannot open file specified");
        }
        Menu = none;
        filename = "";
        break;
        default:
        if(Menu != none){
          filename += C; 
        } else {
          filename = "";
        }
        break;
      }
    }
  } else {
    delay(1500);
    Serial.print("Initializing SD card on port: ");
    Serial.print(PA4);
    Serial.print("...");
    if (!SD.begin(PA4)) {
      Serial.println("initialization failed!");
      isSuccess = false;
      return;
    }
    isSuccess = true;
    Serial.println("initialization done.");
  }
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}
