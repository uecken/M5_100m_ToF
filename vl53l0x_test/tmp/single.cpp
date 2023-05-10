/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

#include <M5StickCPlus.h>
#include <Wire.h>
#define HIGH_SPEED
//#define HIGH_ACCURACY

uint16_t distance=10000;
uint16_t min_distance=10000;

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


void setup()
{ M5.begin();
  Serial.begin(115200);
  Wire.begin(32,33);
  initDisplay();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }


Serial.println(sensor.getMeasurementTimingBudget());
//Serial.printf("%u,%u,%u,%u,%u,%u",sensor.getSequenceStepTimeouts(&enables, &timeouts);)
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  //sensor.setMeasurementTimingBudget(20000);
  sensor.setMeasurementTimingBudget(18000); //if wraparound enabled >=17000 
  //sensor.setMeasurementTimingBudget(12000); //if wraparound disabled >=12000
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif

  sensor.setSignalRateLimit(0.5); //default 0.25mcps. 低い程 微弱信号を検知する.

  //Longer periods increase the potential range of the sensor.
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12); //Pre: 12 to 18 (initialized to 14 by default)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8); //Final: 8 to 14 (initialized to 10 by default)
}

uint16_t previous_time;
uint16_t time_diff;
void loop()
{

  previous_time = millis();
  distance = sensor.readRangeSingleMillimeters();
  time_diff = millis() - previous_time;
  //delay(8);
  Serial.printf("%u,%u \n",time_diff,distance);
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  if( distance < min_distance) min_distance = distance;
  updateDisplay();

  
} 