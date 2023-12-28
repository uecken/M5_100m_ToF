
/*** ESP32 Async web server
Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-async-web-server-espasyncwebserver-library/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#define CLI_XIAO_ESP32C3
#if defined(CLI_XIAO_ESP32C3)

  //#define BUZZER_PIN D7 //Default 10. D9 is bad due to internal pulll-up. D6 is noisy at first buzzer.
  #define BUTTON_PIN 8
  //#define BUTTON_PIN_D9 D9
  #define LED_PIN 10 //Digital out Lowにしないと何故か1V程度ある
  /*
  //#define BUTTON_PIN_ D1
  #define LDO_PIN D1
  #define Vcc_MONITOR_PIN D3
  //#define DEEPSLEEP_WAKEUP_PIN D2
  #define PERIPHERAL_POWER_PIN D2
  #define ULTRASONIC_SIG_PIN D0
  #define SD1306_SDA D4
  #define SD1306_SCL D5
  //#define UART_PIN D6
  */


  //#define YOBI_PIN D9
  //static constexpr int LED_PIN = D6;
  //static constexpr int BUEER_PIN = D9;
  //static constexpr int BUTTON_PIN = D7;

  //#include <Wire.h>
  //#include "SSD1306Wire.h"
  //SSD1306Wire display(0x3c, SDA, SCL);


#endif


//#include <M5StickC.h>
//#define m5cp
//#define esp32s3
/*
#if defined (m5cp)
#include <M5StickCPlus.h>
#elif defined(esp32s3)
#include <Wire.h>
#endif
*/


/*#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
*/

/*
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
const char* ssid = "ESPAsyncWebServer";
const char* password = "";

//=====Web Server Setting=====
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";
const char* PARAM_INPUT_3 = "rssi";
AsyncWebServer server(80);
String outputState(int output);

*/

//#include <VL53L0X.h>
#include "Ultrasonic.h"
#if defined(VL53L0X_h)
  VL53L0X sensor; 
  TFT_eSprite img = TFT_eSprite(&M5.Lcd); 
#elif defined(Ultrasonic_H)
  Ultrasonic ultrasonic(33);
#endif

//#include <Ticker.h>
//Ticker blinker;




#if defined(m5cp)
void LCD_state_init();
void LCD_state_stop();
void LCD_state_start();
void LCD_state_lap();
#endif

//void LED_ONOFF();
//void LED_ONOFF_Reverse(int LED_PIN);


String measureState= "init";
String measureStatePrev ="";
String vl53l0xStarterState= "init";
String vl53l0xStopperState= "init";
unsigned long elapsed_time_ms = 0;
float lap_time_ms = 0;
String rssi;

#if defined(m5cp)
int LED_PIN = 10;
int LED_PIN2 = 26;
#elif defined(CLI_XIAO_ESP32C3)
//int LED_PIN = 10;
#endif

boolean DEBUG_DISTANCE = true;
boolean HTTP_TIME_DEBUG = true;


String outputState(int output){  
  if(digitalRead(output)){
    return "checked";
  }
  else {
    return "";
  }
}




//======vl53l0x========
int Duration;
uint16_t Distance;
unsigned long startMillis,stopMillis,millisPrevious=0;
boolean DEBUG_vl53l0x = 1;  //Serial Load is Heavy. If ON, sometimes unknown error happend. unknown character were printed.
#if defined(VL53L0X_h)
  int distanceThreshold = 1000; //mm
  int distanceThresholdLower = 200; //mm
  unsigned long millisPerRead = 20;
#elif defined(Ultrasonic_H)
  unsigned long millisPerRead = 10;
  int distanceThreshold = 1000; //mm
  int distanceThresholdLower = 5; //mm  
#endif

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

void setup(){
  #if defined(m5cp)
  M5.begin();
  
  M5.Lcd.setRotation(3);              // 画面の向きを変更（右横向き）Change screen orientation (left landscape orientation).
  M5.Axp.ScreenBreath(9);            // 液晶バックライト電圧設定 LCD backlight voltage setting.
  M5.Lcd.fillScreen(BLACK);           // 画面の塗りつぶし　Screen fill.
  LCD_state_init();
  #elif defined(CLI_XIAO_ESP32C3)


  #endif


  Serial.begin(115200);

  //WiFi感度劣化対策　https://twitter.com/uecken/likes
  //GPIO LOW出力で改善
  //pinMode(0,OUTPUT);
  //digitalWrite(0,LOW);  

  
  //-----Digital Out for Debug-----
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); //M5は初期値High、ESP32は初期値Low
  pinMode(BUTTON_PIN, INPUT); //XIAOはINPUT_PULLUP使えない?


#if defined(VL53L0X_h)
  //Init vl53l0x
  Wire.begin(0, 26, 100000);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    img.setCursor(10, 10);
    img.print("Failed");
    img.pushSprite(0, 0);
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.setMeasurementTimingBudget(20000); //50FPS.https://forum.pololu.com/t/high-speed-with-vl53l0x/16585/5
  sensor.startContinuous();
#endif
  startMillis = millis();
}


#if defined(m5cp)
void LCD_state_start(){
      M5.Lcd.setCursor(1, 65, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("State: Start");
      M5.Lcd.setCursor(1, 85, 2);
      M5.Lcd.printf("RSSI:%s",rssi);
}

void LCD_state_lap(){
      //M5.Lcd.setCursor(1, 65, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(1, 20, 2); 
      M5.Lcd.printf("Lap-Time:%4.2f s  ",int(lap_time_ms)/1000.0);
      M5.Lcd.setCursor(1, 105, 2);
      M5.Lcd.printf("RSSI:%s (lapper)   ",rssi);
}


void LCD_state_init(){
  LCD_state_stop();
}

void LCD_state_stop(){
      M5.Lcd.setCursor(1, 1, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("Stop-Time:%4.2f s  ",int(elapsed_time_ms)/1000.0);
      M5.Lcd.setCursor(1, 20, 2); 
      M5.Lcd.printf("Lap-Time:%4.2f s  ",int(lap_time_ms)/1000.0);
      M5.Lcd.println();
      M5.Lcd.printf("Stop-Distance:%4d mm  ",int(Distance));
      M5.Lcd.setCursor(1, 65, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("State: Stop  ");
      M5.Lcd.setCursor(1, 85, 2);
      M5.Lcd.printf("RSSI:%s (starter)   ",rssi);
}
#endif

/*
void LED_ONOFF(){
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}


void LED_ONOFF_Reverse(int LED_PIN){
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
}
*/

long millisPrevious_Distance_dbg = 0;
int Distance_prev = 0;
long Distance_dbg_intervalms = 2000l;

boolean button_state_prev;
boolean button_state;

void loop() {
  delay(1);
  if(digitalRead(BUTTON_PIN)==true && button_state_prev==false){
    measureState="measuring";
  } 
  button_state_prev = digitalRead(BUTTON_PIN);

  if((millis() - millisPrevious) > millisPerRead){
    //===Measure Ultra Sonic===
    
    if(millisPrevious > 5000){ //初回スタート時間は5秒経過以降
      //===HTTP Get Check===
      //start received by http-get
      if ((measureStatePrev=="init"||measureStatePrev=="measured") && measureState=="measuring") {
          startMillis = millis();
          lap_time_ms = 0;
          digitalWrite(LED_PIN, LOW);
          #if defined(m5cp)
          LCD_state_start();
          #endif
      }

      //===Distance Check===
      if (measureState=="measuring" || (millis() - millisPrevious_Distance_dbg) > Distance_dbg_intervalms ) {
        portENTER_CRITICAL_ISR(&mutex);
        #if defined(VL53L0X_h)
          Distance = sensor.readRangeContinuousMillimeters();
        #elif defined(Ultrasonic_H)       
          Distance = ultrasonic.MeasureInCentimeters()*10; // two measurements should keep an interval
        #endif
        portEXIT_CRITICAL_ISR(&mutex);
      }

      if (Duration>0) {
       if(DEBUG_vl53l0x){
         Serial.print(Distance);
         Serial.println(" mm");
       }
      }
    
      //Ultra Sonic Debug
      if(DEBUG_DISTANCE){
        Serial.print(millis() - millisPrevious);
        Serial.print(",");
        Serial.println(Distance);
      }

      if((millis() - millisPrevious_Distance_dbg) > Distance_dbg_intervalms ){
        #if defined(m5cp)
        M5.Lcd.setCursor(1, 50, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("Distance: %d mm", Distance);
        #endif
        millisPrevious_Distance_dbg = millis();
      }

  
      //===HTTP Get Check===
      //stop received by http-get
      if (measureStatePrev=="measuring" && measureState=="measured") {
          stopMillis = millis();
          digitalWrite(LED_PIN, HIGH);  
          #if defined(m5cp)
          LCD_state_stop();
          #endif
          elapsed_time_ms = stopMillis - startMillis;
          if(DEBUG_vl53l0x){
            Serial.print(startMillis);
            Serial.print(" ");
            Serial.println(stopMillis);
            Serial.print(" ");
            Serial.println(String(stopMillis - startMillis) + "ms");
            Serial.print(Distance);
            Serial.println(" mm");
          }
          
          //M5.Lcd.startWrite();
          //M5.Lcd.fillScreen(BLACK);
          //blinker.once_ms(10,LED_ONOFF);
          //M5.Lcd.endWrite();
          //M5.update();
      }

      //reset received by http-get
      else if (measureStatePrev=="measured" && measureState=="init") {
          #if defined(m5cp)
          LCD_state_start();
          #endif
      }
      
      //obstacle detected
      //else if(Distance < distanceThreshold && distanceThresholdLower < Distance){
      else if(abs(Distance - Distance_prev) > 300){
        if(measureState=="measuring"){
          //stop timer
          stopMillis = millis();
          elapsed_time_ms = stopMillis - startMillis;
          digitalWrite(LED_PIN, HIGH); 
          #if defined(m5cp)
          LCD_state_stop();
          #endif

          if(DEBUG_vl53l0x){
            Serial.print(startMillis);
            Serial.print(",");
            Serial.println(stopMillis);
            Serial.print(",");
            Serial.print(String(stopMillis - startMillis) + "ms,");
            Serial.print(Distance);
            Serial.println(" mm");
          }

          measureState = "measured";
          vl53l0xStopperState = "detected";
          
          //blinker.once_ms(10,LED_ONOFF);
        }
      }
  
      //経過時間計算
      if(measureState=="measuring"){
        elapsed_time_ms = millis() - startMillis;
      }
    
    Distance_prev = Distance;
    measureStatePrev = measureState;
    }
    millisPrevious = millis();
  }
}
