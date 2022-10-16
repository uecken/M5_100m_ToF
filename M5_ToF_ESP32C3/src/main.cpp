//=====Device Setting=====
//#define AP
//#define CLI
#define CLI_M5STAMP
//#define CLI_M5ATOM

#ifdef AP
  const char *ssid = "ESPAsyncWebServer";
#elif defined CLI
#elif defined CLI_M5STAMP
#else
#endif


//=====WiFi=====
#include <WiFi.h>
#include <esp_wifi.h>
static const char WIFI_SSID[] = "ESPAsyncWebServer";
static const char WIFI_PASSPHRASE[] = "";
static const char SERVER[] = "192.168.4.1";
int c = esp_wifi_set_max_tx_power(80); //20dBm https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
String rssi;

#define TASK_NAME_WIFI "WiFiCheckTask"
#define TASK_SLEEP_WIFI 1000 //1s delay
static void WiFiCheckLoop(void* arg);

//=====US=====
#include "Ultrasonic.h"
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
Ultrasonic ultrasonic(0);
uint16_t distance;
uint16_t distance_prev;
static uint16_t distance_rel_th_ms = 300;
String measure_state;

#define TASK_DEFAULT_CORE_ID 0 //ESP32C3Uは1Coreのため
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_US "UltrasonicTask"
#define TASK_SLEEP_ULTRASONIC 15 //10ms delay
static void UltrasonicLoop(void* arg);

//====LED=====
#include <Adafruit_NeoPixel.h>
#define LED_PIN 2
static int BUZZER_PIN = 20;
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);
void LED_Buzzer_ONOFF();


//=====HTTP Setting=====
#include <HTTPClient.h>
boolean WAV_DEBUG = false;
boolean SOUND_DEBUG = false;
boolean HTTP_DEBUG = false;
boolean TIME_DEBUG = true;
String payload;
void httpGetUltraSonic(String mode,boolean make_sound);


//=====M5StampC3U=====
#include <M5Unified.h>

void setup() {
  M5.begin();
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring WiFi...");

  #ifdef AP
  #elif defined(CLI)
  #elif defined(CLI_M5STAMP)
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN,LOW);
    pixels.begin();
    WiFi.mode(WIFI_STA);
    if (WIFI_SSID[0] != '\0')WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
    else WiFi.begin();
  #else
  #endif
  

  xTaskCreatePinnedToCore(UltrasonicLoop, TASK_NAME_US, TASK_STACK_DEPTH, NULL, 2, NULL, TASK_DEFAULT_CORE_ID);
  xTaskCreatePinnedToCore(WiFiCheckLoop, TASK_NAME_WIFI, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);

}


void Buzzer_ONOFF(int buzzer_cnt){
  #ifdef CLI_M5STAMP
  for(int i=0; i<buzzer_cnt; i++){
    digitalWrite(BUZZER_PIN,HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN,LOW);
    delay(100);
  }
  #endif
}

String sense_mode = "start";
bool long_press_1s;
bool long_press_2s;

void loop(){
  M5.update();
  delay(100);

  if(sense_mode =="lap" && M5.BtnA.wasReleased()){
    if(long_press_2s){//長押し処理(2s)
      Buzzer_ONOFF(3);
      sense_mode = "start";
      ESP.restart(); //何故か徐々にHTTPの返答が遅くなるため、Restartさせる
    } else{//短押し処理
    //==http send test===
      httpGetUltraSonic("start",true);
      LED_Buzzer_ONOFF();
      //Buzzer_ONOFF(1);
    }
  }
  else if(sense_mode =="start" && M5.BtnA.wasReleased()){
    if(long_press_2s){//長押し処理(2s)
      sense_mode = "lap";
      Buzzer_ONOFF(3);
    }
    else if(long_press_1s){//長押し処理(1s)
    // Onyourmark set pistol
      Buzzer_ONOFF(2);
      //delay(10000); //10秒待機
      delay(1000); //10秒待機

      Buzzer_ONOFF(3); // On your marks
      //delay(16000); //16秒待機
      delay(1000); //16秒待機
      Buzzer_ONOFF(2);//  set...
      delay(1800 + (rand() % 5)*100); //1.8~2.2秒待機
      
      httpGetUltraSonic(sense_mode,true);
      Buzzer_ONOFF(1); // BAN! (pistor)

    }
    else{//短押し処理
    //==http send test===
      httpGetUltraSonic(sense_mode,true);
      LED_Buzzer_ONOFF();
    }
  }

  long_press_1s = M5.BtnA.pressedFor(1000);
  long_press_2s = M5.BtnA.pressedFor(2000);
}


static void UltrasonicLoop(void* arg){
  while (1) {
    uint32_t entryTime = millis();
    if(sense_mode == "lap"){
      portENTER_CRITICAL_ISR(&mutex);
      distance = ultrasonic.MeasureInCentimeters()*10;
      portEXIT_CRITICAL_ISR(&mutex);
      
      if(Serial.available())Serial.println("US measuring:"+String(distance));
      if((distance_prev - distance) >  distance_rel_th_ms){//距離が短くなった時のみ
          //measure_state = "measuring";
          httpGetUltraSonic(sense_mode,true);
          if(Serial.available())Serial.println(String(distance)+","+String(millis()));
          LED_Buzzer_ONOFF();
          vTaskDelay(1000);
          //distance_prev = ultrasonic.MeasureInCentimeters()*10;
      }else if((distance_prev - distance) <  distance_rel_th_ms){
      }

      distance_prev = distance;
    }
    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_ULTRASONIC  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);

  }
}


static void WiFiCheckLoop(void* arg){
  while (1) {
    uint32_t entryTime = millis();

    if(WiFi.status() != WL_CONNECTED){
      Serial.println("Reconnecting to WiFi...");
      WiFi.reconnect();
      //Automatically Reconnect to AP...
    }else if(WiFi.status() == WL_CONNECTED){
      rssi = String(WiFi.RSSI());
    }

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_WIFI  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}


void httpGetUltraSonic(String mode, boolean make_sound){
    long tmp_start_millis = millis();
    HTTPClient http;
    http.begin("http://192.168.4.1/update?output=vl53l0x_"+mode+"&state=0&rssi="+String(rssi)); // starter
    //http.begin("http://192.168.4.1/update?output=vl53l0x_stop&state=0&rssi="+String(rssi)); // stopper
    int httpCode = http.GET(); // Make the request
    long tmp_stop_millis = millis();
    if(TIME_DEBUG) Serial.println("httpget:"+String(tmp_stop_millis-tmp_start_millis) + "ms");

    if (httpCode > 0) { //Check for the returning code
        payload = http.getString();
        if(HTTP_DEBUG) Serial.println(httpCode);
        if(HTTP_DEBUG) Serial.println(payload);
      }
    else {
      Serial.println("Error on HTTP request");
      Serial.println(httpCode);
    }
    http.end(); //Free the resources
}


void LED_Buzzer_ONOFF(){
  #ifdef CLI_M5STAMP
    digitalWrite(BUZZER_PIN,HIGH);
    pixels.setPixelColor(0, pixels.Color(0, 255, 255));    //Colorメソッド内の引数の順番は赤、緑、青
    pixels.show();delay(100);
    digitalWrite(BUZZER_PIN,LOW);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));    //明るさを3原色全て0にして消灯
    pixels.show();delay(100);
  #else
    digitalWrite(BUZZER_PIN,HIGH);digitalWrite(LED_PIN, HIGH);delay(100);
    digitalWrite(BUZZER_PIN,LOW); digitalWrite(LED_PIN, LOW);delay(100);
  #endif
}