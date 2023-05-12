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
#elif defined (Adafruit_VL53L0X_Sensor)
  #include <Adafruit_VL53L0X.h>
  Adafruit_VL53L0X lox;
  
#endif






#define SENSOR_NUM  2 // 使用するセンサーの数
#define ADDRESS_DEFAULT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFAULT + 2)
/**
 * 各センサのXSHUTへ接続されているGPIOの配列
 */
const int GPIO_MASK_ARRAY[SENSOR_NUM] = {25, 26};
VL53L0X gSensor[SENSOR_NUM]; // 使用するセンサークラス配列



uint16_t distance1=10000;
uint16_t min_distance1=10000;
uint16_t distance2=10000;
uint16_t min_distance2=10000;
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

void updateDisplay(VL53L0X sensor){
  M5.update();


  if(sensor.xshut_pin == 25) M5.Lcd.setCursor(5, 5);  M5.Lcd.setTextColor(WHITE, BLACK); //0x29
  if(sensor.xshut_pin == 26) M5.Lcd.setCursor(5, 20);  M5.Lcd.setTextColor(WHITE, BLACK); //0x30

  M5.Lcd.printf("%u,%u,%u        ",sensor.min_distance,sensor.distance,sensor.meas_time_diff);


  if(gSensor[0].distance < 500 & gSensor[1].distance < 500 ){
    M5.Lcd.setCursor(5, 35);
    M5.Lcd.printf("Detected.");
  }

  if ( M5.BtnA.wasPressed() ) {
    gSensor[0].min_distance=10000;
    gSensor[1].min_distance=10000;
    M5.Lcd.setCursor(5, 35);
    M5.Lcd.printf("              ");

  }
}
#endif


void setup() {
  #if defined (M5StickC)
  M5.begin();
  initDisplay();
  Wire.begin(SDA_PIN,SDL_PIN);
  #elif defined (XIAO_ESP32C3)
  Wire.begin(SDA_PIN,SDL_PIN);
  #endif
  Serial.begin(115200);


  // まず全てのGPIOをLOW
  for (int i = 0; i < SENSOR_NUM; i++)
  {
    pinMode(GPIO_MASK_ARRAY[i], OUTPUT);
    digitalWrite(GPIO_MASK_ARRAY[i], LOW);
  }

  for (int i = 0; i < SENSOR_NUM; i++) {
    // センサを初期化
    gSensor[i].xshut_pin = GPIO_MASK_ARRAY[i];
    pinMode(GPIO_MASK_ARRAY[i], INPUT);
    delay(10);
    if (gSensor[i].init() == true)
    {
      gSensor[i].setTimeout(1000);
      gSensor[i].setSignalRateLimit(2); 
      gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 12); //Pre: 12 to 18 (initialized to 14 by default)
      gSensor[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 8); //Final: 8 to 14 (initialized to 10 by default)
      gSensor[i].setMeasurementTimingBudget(20000); 
      gSensor[i].startContinuous();
      int address = ADDRESS_00 + (i * 1);
      gSensor[i].setAddress(address);
    }
    else
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" error");
    }
  }
}




  void loop()
{
  for (int i = 0; i < SENSOR_NUM; i++) {

    previous_time = millis();
    delay(10/SENSOR_NUM);
    gSensor[i].prev_meas_time = previous_time;
    uint8_t address = gSensor[i].getAddress();
    gSensor[i].distance = gSensor[i].readRangeContinuousMillimeters();
    
    if( gSensor[i].distance < gSensor[i].min_distance) gSensor[i].min_distance = gSensor[i].distance;
    time_diff = millis() - previous_time;
    gSensor[i].meas_time_diff = time_diff;
    Serial.printf("%u,%u,%u \n",address,time_diff,gSensor[i].distance);

    if (gSensor[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    #if defined (M5StickC)
    updateDisplay(gSensor[i]);
    #endif

  }
}