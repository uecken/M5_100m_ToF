/* This example shows how to use continuous mode to take
range measurements with the VL53L0X. It is based on
vl53l0x_ContinuousRanging_Example.c from the VL53L0X API.

The range readings are in units of mm. */

//#define XIAO_ESP32C3
#define M5StickC
#if defined (M5StickC)
  #include <M5StickCPlus.h>
  #define SDA_PIN 32 //D4
  #define SDL_PIN 33 //D5
#elif defined (XIAO_ESP32C3)
  //#include <Arduino.h>
  //#define SDA_PIN 6 //D4
  //#define SDL_PIN 7 //D5
#endif

#include <Wire.h>

#define pololu_VL53L0X
//#define Adafruit_VL53L0X_Sensor //cannot perform
#if defined (pololu_VL53L0X)
  #include <VL53L0X.h>
  VL53L0X sensor;
#elif defined (Adafruit_VL53L0X_Sensor)
  #include <Adafruit_VL53L0X.h>
  Adafruit_VL53L0X lox;
  
#endif

/*
#define TASK_DEFAULT_CORE_ID 0 //ESP32C3Uは1Coreのため
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_US "UltrasonicTask"
//#define TASK_SLEEP_VL53L0X 10 //10ms delay
static void VL53L0XLoop(void* arg);
*/
#define TASK_SLEEP_VL53L0X 12 //10ms delay

uint16_t distance=10000;
uint16_t min_distance=10000;
uint16_t previous_time;
uint16_t time_diff;

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
  M5.Lcd.printf("%u,%u,%u           ",distance, min_distance,time_diff);

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
  Wire.begin(SDA_PIN,SDL_PIN);

  #elif defined (XIAO_ESP32C3)
  Wire.begin(SDA_PIN,SDL_PIN);

  #endif

  Serial.begin(115200);


  #if defined (pololu_VL53L0X)
  
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  sensor.setSignalRateLimit(2); //default 0.25mcps. 低い程 微弱信号を検知する. 
                                //5で300mm程度まで反応
                                //2で500mmまで反応、30秒で屋外誤り無し
                                //1.52で500mmまで反応、誤りあり
                                //0.5で500mmまで反応、誤りあり

  //Longer periods increase the potential range of the sensor.
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12); //Pre: 12 to 18 (initialized to 14 by default)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8); //Final: 8 to 14 (initialized to 10 by default)

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  
  //sensor.setMeasurementTimingBudget(17800); // tcc OFFの場合. 測定間隔が誤測定し易いが、測定回数が多いためと推測する
  sensor.setMeasurementTimingBudget(20000); 
  //sensor.setMeasurementTimingBudget(33000); 

  sensor.startContinuous();
  //sensor.startContinuous(10); //10でも20でも30ms間隔のため、引数無しにしてsetMeasurementTimingBudget(20000)が良い。

  #elif defined (Adafruit_VL53L0X_Sensor)
  lox.startRangeContinuous();
  #endif
}



void loop()
{
  uint32_t entryTime = millis();
  
  previous_time = millis();
  #if defined (pololu_VL53L0X)
    distance = sensor.readRangeContinuousMillimeters(); //continuousはsetMeasurementTimingBudgetがある程度効く
    //distance = sensor.readRangeSingleMillimeters();
  #elif defined (Adafruit_VL53L0X_Sensor)
    if (lox.isRangeComplete()) distance = lox.readRange();
  #endif
  time_diff = millis() - previous_time;
  
  //if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  if( distance < min_distance) min_distance = distance;

  #if defined (M5StickC)
  updateDisplay();
  #endif

  uint16_t time_diff2 = millis() - previous_time;
  Serial.printf("%u,%u,%u \n",time_diff,distance,time_diff2);


  uint32_t elapsed_time = millis() - entryTime; 
  int32_t sleep = TASK_SLEEP_VL53L0X  - elapsed_time;
  vTaskDelay((sleep > 0) ? sleep : 0);

}


