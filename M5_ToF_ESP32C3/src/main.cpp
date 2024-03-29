#include <Arduino.h>
//#include <driver/rtc_io.h>

#define Model_SolarChargerSupercapacita_with_Deepsleep
//#define RST_3V3_and_GND_with_Short
#define DoDeepsleep
#define Model_SolarChargerCR123A_with_Deepsleep
#ifdef  Model_SolarChargerCR123A_with_Deepsleep
  RTC_DATA_ATTR boolean force_deepsleep = false;
#endif

#define CLI_XIAO_ESP32C3
//#define CLI_M5STAMP

#if defined(CLI_XIAO_ESP32C3)
  #define LED_PIN D10 //Digital out Lowにしないと何故か1V程度ある
  #define BUZZER_PIN D4 //Default D10. D9 is bad due to internal pulll-up. D6 is noisy at first buzzer.
  #define BUTTON_PIN D7
  #define BUTTON_PIN_D8 D8
  //#define BUTTON_PIN_ D1
  #define LDO_PIN D1
  #define Vcc_MONITOR_PIN D3
  #define DEEPSLEEP_WAKEUP_PIN D2
  //#define YOBI_PIN D9
  //static constexpr int LED_PIN = D6;
  //static constexpr int BUEER_PIN = D9;
  //static constexpr int BUTTON_PIN = D7;
#elif defined(CLI_M5STAMP)
  #define LED_PIN 2
  static int BUZZER_PIN = 20;
#else
#endif


//=====WiFi=====
#include <WiFi.h>
#include <esp_wifi.h>
static const char WIFI_SSID[] = "ESPAsyncWebServer";
static const char WIFI_PASSPHRASE[] = "";
static const char SERVER[] = "192.168.4.1";
int a = esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_11B );
int c = esp_wifi_set_max_tx_power(80); //20dBm https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
//int c = esp_wifi_set_max_tx_power(66); //16dBm https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
//int c = esp_wifi_set_max_tx_power(56); //16dBm https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
int d = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_2M_S);  // Sensitibity -96dBm
       //Long->192us,Short->96us, Ref:https://www.silex.jp/blog/wireless/2013/12/post-8.html
       //Ref: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html#_CPPv416wifi_interface_t
String rssi;

#define TASK_NAME_WIFI "WiFiCheckTask"
#define TASK_SLEEP_WIFI 10000 //1s delay
static void WiFiCheckLoop(void* arg);

//=====US=====
#include "Ultrasonic.h"
portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;
Ultrasonic ultrasonic(0);
uint16_t distance;
uint16_t distance_prev;
static uint16_t distance_rel_th_ms = 200;
String measure_state;

#define TASK_DEFAULT_CORE_ID 0 //ESP32C3Uは1Coreのため
#define TASK_STACK_DEPTH 4096UL
#define TASK_NAME_US "UltrasonicTask"
#define TASK_SLEEP_ULTRASONIC 10 //10ms delay
static void UltrasonicLoop(void* arg);




//=====HTTP Setting=====
#include <HTTPClient.h>
boolean DEBUG = true; //WiFi
boolean WAV_DEBUG = false;
boolean SOUND_DEBUG = false;
boolean HTTP_DEBUG = false;
boolean TIME_DEBUG = false;
boolean US_DEBUG = false;
String payload;
boolean httpGetUltraSonic(String mode,boolean make_sound);


//=====M5StampC3U=====
#if defined(CLI_M5STAMP)
#include <M5Unified.h>
  //====LED=====
  #include <Adafruit_NeoPixel.h>
  Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);
#endif
void LED_Buzzer_ONOFF();
void LED_ONOFF(int LED_GPIO);
void magnetSwitchStarterAfterPowerOn();
float get_battery_voltage();
const float SLEEP_VCC_THRESHOLD = 3.3;
void gotoDeepSleep();

//======= Timer ======
int timer_after_wifi_start = 2000;


String sense_mode = "vl53l0x_start";
bool long_press_1s;
bool long_press_2s;

void setup() {
  #if defined(CLI_M5STAMP)
  M5.begin();
  #endif
  Serial.begin(115200);
  Serial.println();
  
  #if defined(CLI_XIAO_ESP32C3)
  /*
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason)
    {
      case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
      case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
      case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
      case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
      default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); 
                #if defined (RST_3V3_and_GND_with_Short)
                  gotoDeepSleep();
                #endif
                break;
    }
  */

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN,HIGH);
    delay(100);
    digitalWrite(LED_PIN,LOW);

    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN,LOW);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(BUTTON_PIN_D8, INPUT_PULLUP);
    
    pinMode(D0,INPUT_PULLUP);
    pinMode(LDO_PIN,OUTPUT);
    digitalWrite(LDO_PIN,LOW); //!!!! LDO OFF !!!! v0.3
    digitalWrite(LDO_PIN,HIGH); //!!!! LDO ON !!!! v0.4
    

    //pinMode(D1,OUTPUT);
    //digitalWrite(D1,HIGH);

    pinMode(Vcc_MONITOR_PIN,INPUT);
    
    //pinMode(D6,OUTPUT); //D6は正常に使えない
    //digitalWrite(D6,HIGH);

    //pinMode(YOBI_PIN, OUTPUT);
    //digitalWrite(YOBI_PIN,LOW);
    
    //WiFi.mode(WIFI_STA);
    //WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
    //if (WIFI_SSID[0] != '\0')WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
    //else WiFi.begin();
    

    //pinMode(10,INPUT_PULLDOWN);
    //if(digitalRead(10) == HIGH) sense_mode = "lap";
    
  #elif defined(CLI_M5STAMP)
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN,LOW);
    pixels.begin();
    WiFi.mode(WIFI_STA);
    if (WIFI_SSID[0] != '\0')WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
    else WiFi.begin();

    pinMode(10,INPUT_PULLDOWN);
    if(digitalRead(10) == HIGH) sense_mode = "lap";

  #else
  #endif

  //#if defined(Model_SolarChargerCR123A_with_Deepsleep)
  //pinMode(D2,INPUT_PULLDOWN);
  //pinMode(D2,INPUT);

  //digitalWrite(GPIO_NUM_5, LOW);
  //#define DEBUG_BatteryVoltageRead
  #if defined(DEBUG_BatteryVoltageRead)
  delay(30000);
  float aaaa = get_battery_voltage();
  #endif

  //if(digitalRead(D2)==LOW){
  //if(digitalRead(D2)==HIGH || (get_battery_voltage() < SLEEP_VCC_THRESHOLD)){
  #if (defined(Model_SolarChargerCR123A_with_Deepsleep) || defined(Model_SolarChargerSupercapacita_with_Deepsleep)) && defined(DoDeepsleep) && !defined(RST_3V3_and_GND_with_Short)
  if((get_battery_voltage() < SLEEP_VCC_THRESHOLD)){
    /*
    ::esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    
    
    pinMode(GPIO_NUM_5,OUTPUT);   
    digitalWrite(GPIO_NUM_5, HIGH);
    gpio_hold_en(GPIO_NUM_5);

    if(digitalRead(D2)==HIGH)
      ::esp_deep_sleep_enable_gpio_wakeup(BIT(D2), ESP_GPIO_WAKEUP_GPIO_LOW);
    else{
      ::esp_deep_sleep_enable_gpio_wakeup(BIT(D2), ESP_GPIO_WAKEUP_GPIO_HIGH);
    }
    */

    //pinMode(GPIO_NUM_5,OUTPUT);//D3
    //digitalWrite(GPIO_NUM_5, HIGH);
    //gpio_hold_en(GPIO_NUM_5);
    
    //gpio_hold_en(GPIO_NUM_3); //D0, GPIO_NUM3
    //gpio_deep_sleep_hold_en();

    // Cannot import  #include <driver/rtc_io.h>
	  //rtc_gpio_set_direction((gpio_num_t), RTC_GPIO_MODE_INPUT_ONLY);
  	//rtc_gpio_pulldown_en((gpiao_num_t)PIN_LORA_DIO_1);
  	//rtc_gpio_set_direction((gpio_num_t)GPIO_NUM_3 RTC_GPIO_MODE_OUTPUT_ONLY);
	  //rtc_gpio_set_level((gpio_num_t)GPIO_NUM_3, HIGH);
    

    //force_deepsleep = false;
    //::esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    //::esp_deep_sleep_enable_gpio_wakeup(BIT(DEEPSLEEP_WAKEUP_PIN), ESP_GPIO_WAKEUP_GPIO_HIGH);// Lowを使う時はプルアップしないと上手く行かない。
    //::esp_deep_sleep_start();
    gotoDeepSleep();
  }
  #endif


  //#define SolarPowerOFF_and_BatteryPerform
  #if defined(SolarPowerOFF_and_BatteryPerform)
  pinMode(D3,OUTPUT);//D3
  digitalWrite(D3, LOW);
  #endif
  


  /*
  if(force_deepsleep){
    force_deepsleep = false;
    ::esp_deep_sleep_enable_gpio_wakeup(BIT(D2), ESP_GPIO_WAKEUP_GPIO_LOW);
    ::esp_deep_sleep_start();
  }else{
    */
    LED_Buzzer_ONOFF();
    //vTaskDelay(50);
    //LED_ONOFF(LED_PIN);
    //vTaskDelay(3000); //When bootup, wait a few seconds.
    if(DEBUG)Serial.println("Configuring WiFi...");
    delay(1000);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
    delay(timer_after_wifi_start); //wait until wifi connected.
    xTaskCreatePinnedToCore(WiFiCheckLoop, TASK_NAME_WIFI, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
    if(WiFi.status() == WL_CONNECTED) if(DEBUG)Serial.println("WiFi connected");

    //xTaskCreatePinnedToCore(UltrasonicLoop, TASK_NAME_US, TASK_STACK_DEPTH, NULL, 2, NULL, TASK_DEFAULT_CORE_ID);

    if(digitalRead(D0)==HIGH){
      magnetSwitchStarterAfterPowerOn();    
    }
  //}

  //Deep-sleep
  #if (defined(Model_SolarChargerCR123A_with_Deepsleep) || defined(Model_SolarChargerSupercapacita_with_Deepsleep)) && defined(DoDeepsleep)
  //if(digitalRead(DEEPSLEEP_WAKEUP_PIN)==LOW){
    //force_deepsleep = true;
    gotoDeepSleep();
  //}
  #endif


  //LED_Buzzer_ONOFF();
}

void magnetSwitchStarterAfterPowerOn(){
  LED_Buzzer_ONOFF();
  if(httpGetUltraSonic(sense_mode,true)){
    LED_Buzzer_ONOFF();
  }
}


void Buzzer_ONOFF(int buzzer_cnt){
  #if defined(CLI_M5STAMP) || defined(CLI_XIAO_ESP32C3)
  for(int i=0; i<buzzer_cnt; i++){
    digitalWrite(BUZZER_PIN,HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN,LOW);
    delay(100);
  }
  #endif
}

float get_battery_voltage(){
  uint32_t Vbatt = 0;
  uint8_t Nsmp =16;
  for(int i = 0; i < Nsmp; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(Vcc_MONITOR_PIN); // ADC with correction   
  }
  float Vbattf = 2 * Vbatt / Nsmp / 1000.0;     // attenuation ratio 1/2, mV --> V
  if(DEBUG)Serial.println(Vbattf, 4);
  return Vbattf;
}


void loop(){

  delay(300);
  //digitalWrite(BUZZER_PIN,LOW);

#if defined(CLI_XIAO_ESP32C3)
  if(sense_mode =="vl53l0x_start" && !digitalRead(BUTTON_PIN)){
      /*
      if(long_press_1s){//長押し処理(1s)
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
      else */
      if(httpGetUltraSonic(sense_mode,true)){
        LED_Buzzer_ONOFF();
      }else{
      }
  }else if(sense_mode =="vl53l0x_start" && !digitalRead(BUTTON_PIN_D8)){
      if(httpGetUltraSonic("led_onoff",true)){
        LED_Buzzer_ONOFF();
      }else{
      }
  }

#elif defined(CLI_M5STAMP)
  M5.update();
  if(sense_mode =="lap" && M5.BtnA.wasReleased()){
    if(long_press_2s){//長押し処理(2s)
      Buzzer_ONOFF(3);
      sense_mode = "start";
      //ESP.restart(); //何故か徐々にHTTPの返答が遅くなるため、Restartさせる
    } else{//短押し処理
    //==http send test===
      httpGetUltraSonic(sense_mode,true);
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
#endif
}


static void UltrasonicLoop(void* arg){
  static uint32_t timeout_us = TASK_SLEEP_ULTRASONIC*1000;
  while (1) {
    uint32_t entryTime = millis();
    if(sense_mode == "lap"){
      portENTER_CRITICAL_ISR(&mutex);
      distance = ultrasonic.MeasureInMillimeters(timeout_us);
      portEXIT_CRITICAL_ISR(&mutex);
      
      if(US_DEBUG)Serial.println("US measuring:"+String(distance)+",ElapsedTime:"+String(millis() - entryTime)+",Millis:"+String(millis()));
      if((distance != 0 && (distance_prev - distance) >  distance_rel_th_ms) || (distance_prev == 0 && (distance_prev < distance))){//距離が短くなった時のみ && 物体未検出から検出に映った時
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

  /*
  while ( WiFi.status() != WL_CONNECTED ){
   delay( 500 );
   Serial.print(".");
 }
 */
  vTaskDelay(timer_after_wifi_start + 1000);

  while (1) {
    uint32_t entryTime = millis();
    LED_ONOFF(LED_PIN);

    //For SolarCharge. If D2==Low, do solarCharge.  

    #if defined(Model_SolarChargerCR123A_with_Deepsleep)
    /*
    if(digitalRead(D2)==LOW){
      //Deep Sleep and 
      ESP.restart();
    }else if(digitalRead(D2)==HIGH){
      //If D2==High, No sorlar charge, serve power from litium-battery
      //Wakeup from deep-sleep 
      //::esp_deep_sleep_enable_gpio_wakeup(BIT(D2), ESP_GPIO_WAKEUP_GPIO_LOW);
      //::esp_deep_sleep_start();
    }
    */
    #endif

    if(WiFi.status() != WL_CONNECTED){
      /* --- Cannot connect to AP after WiFi OFF and ON. 
      if(DEBUG)Serial.println("Reconnecting to WiFi...");

      WiFi.mode(WIFI_STA);
      WiFi.begin(WIFI_SSID, WIFI_PASSPHRASE);
      vTaskDelay(3000);

      if(WiFi.status() != WL_CONNECTED){
        if(DEBUG)Serial.println("WiFi reconnection failed. Stopping WiFi for battery-saving");

        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
      }else{
          if(DEBUG)Serial.println("WiFi reconnected");
      }
      
      */
      WiFi.mode(WIFI_OFF);
      vTaskDelay(10000);
      ESP.restart();
      //Automatically Reconnect to AP...
    }else if(WiFi.status() == WL_CONNECTED){
      rssi = String(WiFi.RSSI());
    }

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_WIFI  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}


boolean httpGetUltraSonic(String mode, boolean make_sound){
    boolean result;
    long tmp_start_millis = millis();
    HTTPClient http;
    if(WiFi.status() == WL_CONNECTED){
      rssi = String(WiFi.RSSI());
    }

    float vbatt_f = get_battery_voltage();
    char vbatt_c[6];
    snprintf(vbatt_c, 6, "%f", vbatt_f);

    http.begin("http://192.168.4.1/update?output="+mode+"&state=0&rssi="+String(rssi)+"&vbat="+vbatt_c); // starter
    //http.begin("http://192.168.4.1/update?output=vl53l0x_stop&state=0&rssi="+String(rssi)); // stopper
    int httpCode = http.GET(); // Make the request
    long tmp_stop_millis = millis();
    if(TIME_DEBUG && Serial.available()) Serial.println("httpget:"+String(tmp_stop_millis-tmp_start_millis) + "ms");

    if (httpCode > 0) { //Check for the returning code
        payload = http.getString();
        if(HTTP_DEBUG) Serial.println(httpCode);
        if(HTTP_DEBUG) Serial.println(payload);
        result=true;
      }
    else {
      if(Serial.available())Serial.println("Error on HTTP request");
      if(Serial.available())Serial.println(httpCode);
      result=false;
    }
    http.end(); //Free the resources
    return result;
}


void LED_Buzzer_ONOFF(){
  #if defined(CLI_XIAO_ESP32C3)
    digitalWrite(BUZZER_PIN,HIGH);digitalWrite(LED_PIN, HIGH);delay(100);
    digitalWrite(BUZZER_PIN,LOW); digitalWrite(LED_PIN, LOW);delay(100);
    //pass
  #elif defined(CLI_M5STAMP)
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


void LED_ONOFF(int LED_GPIO){
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, HIGH);
    delay(50);
    digitalWrite(LED_GPIO, LOW);
    delay(50);
}


void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}


void gotoDeepSleep(){
    ::esp_deep_sleep_enable_gpio_wakeup(BIT(DEEPSLEEP_WAKEUP_PIN), ESP_GPIO_WAKEUP_GPIO_HIGH);// Lowを使う時はプルアップしないと上手く行かない。
    ::esp_deep_sleep_start();
    //dee
}