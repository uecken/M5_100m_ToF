/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

#define XIAO_ESP32C3
//#define M5StickC
#if defined (M5StickC)
  #include <M5StickCPlus.h>
#elif defined (XIAO_ESP32C3)
  #include <Arduino.h>
#endif

#define SDA_PIN 6 //D4
#define SDL_PIN 7 //D5

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
uint16_t distance=10000;
uint16_t min_distance=10000;

#if defined (M5StickC)
void initDisplay(){
    // 液晶表示初期設定
  M5.Lcd.fillScreen(BLACK); //背景色
  M5.Lcd.setRotation(1);    //画面向き設定（USB位置基準 0：下/ 1：右/ 2：上/ 3：左）
  M5.Lcd.setTextSize(1);    //文字サイズ（整数倍率）
  M5.Lcd.setTextFont(2);    //フォント 1(8px), 2(16px), 4(26px)
                                     //6(36px数字-.:apm), 7(7セグ-.:), 8(75px数字-.:)
}



void updateDisplay(){
  M5.update();
  M5.Lcd.setCursor(5, 5);  M5.Lcd.setTextColor(RED, BLACK); //座標設定(x, y)//(文字色, 背景)
  M5.Lcd.printf("%u,%u   ",distance, min_distance);

  if ( M5.BtnA.wasPressed() ) {
    min_distance=10000;
  }
}
#endif

void setup()
{ 
  #if defined (M5StickC)
  M5.begin();
  initDisplay();
  Wire.begin(32,33);

  #elif defined (XIAO_ESP32C3)
  Wire.begin(SDA_PIN,SDL_PIN);

  #endif

  Serial.begin(115200);
  
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  sensor.setSignalRateLimit(0.5); //default 0.25mcps. 低い程 微弱信号を検知する.

  //Longer periods increase the potential range of the sensor.
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12); //Pre: 12 to 18 (initialized to 14 by default)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8); //Final: 8 to 14 (initialized to 10 by default)

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  
  sensor.setMeasurementTimingBudget(17800); 
  sensor.startContinuous();
}


uint16_t previous_time;
uint16_t time_diff;
void loop()
{
  previous_time = millis();
  distance = sensor.readRangeContinuousMillimeters();
  time_diff = millis() - previous_time;
  Serial.printf("%u,%u \n",time_diff,distance);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  if( distance < min_distance) min_distance = distance;

  #if defined (M5StickC)
  updateDisplay();
  #endif

}
