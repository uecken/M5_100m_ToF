//#include <Arduino.h>
#include <Wire.h> //OLEDとI2C接続するためのライブラリを読み込む 
#include <SSD1306.h>
 
#define SDA 6 //SDAをGPIOの22へ
#define SCL 7 //SCLをGPIOの18へ
 
SSD1306 display(0x3c, SDA, SCL); //0x3cはI2Cアドレス
 
void setup() {
 
  display.init(); //ディスプレイを初期化
  display.drawString(0, 0, "Hello World from ESP32!");
  display.display(); //指定された文字を描く
}
 
void loop() {}