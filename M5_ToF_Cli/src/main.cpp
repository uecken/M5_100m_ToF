
//#include <M5StickC.h>
#include <M5Atom.h>

#include <WiFiMulti.h> 
#include <HTTPClient.h>
#include <esp_wifi.h>

//#include <VL53L0X.h>
#include "Ultrasonic.h"
#if defined(VL53L0X_h)
  VL53L0X sensor; 
  //TFT_eSprite img = TFT_eSprite(&M5.Lcd); 
#elif defined(Ultrasonic_H)
  #ifdef _M5STICKC_H_
    Ultrasonic ultrasonic(33);
  #endif
  #ifdef _M5ATOM_H_
    Ultrasonic ultrasonic(32);
  #endif
#endif
const char* ssid = "ESPAsyncWebServer";
const char* pass = "";

uint16_t Distance;

unsigned long startMillis,stopMillis,millisPrevious;
unsigned long millisPerRead = 20; //10ms以下稀にフリーズする
String measureState= "init";
String measureStatePrev ="";
String vl53l0xState ="init";
int distanceThreshold = 1000; //mm
int distanceThresholdLower = 200; //mm
boolean DEBUG_ULTRASONIC = 0;
unsigned long elapsed_time_ms = 0;
unsigned long httpGetExecTime=0;

unsigned long previousWiFiMillis = 0;
unsigned long currentWiFiMillis = 0;
unsigned long WiFiCheckInterval = 5000;
uint8_t wifiState;

/*
unsigned long previousPollingMillis = 0;
unsigned long currentPollingMillis = 0;
unsigned long PollingInterval = 10010;
*/

String sense_mode = "start";
int LED_GPIO = 10;

WiFiMulti wifiMulti;
String rssi;
String payload;

  boolean WAV_DEBUG = false;
  boolean SOUND_DEBUG = false;
  boolean HTTP_DEBUG = false;
  boolean TIME_DEBUG = true;

void httpGetStatePolling();
void httpGetUltraSonic(String mode,boolean make_sound);
void draw_payload();
void draw_executed_mode(String mode);
void draw_sense_mode(String mode);
void M5C_LED_ON_and_OFF(int ontime);
//void draw_WiFi_State(String state);



#include <AtomEcho.h>
AtomEcho atomecho;

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;


void setup() {
  #ifdef _M5STICKC_H_
  pinMode(10, OUTPUT);
  #elif defined(_M5ATOM_H_)
  pinMode(21, OUTPUT); //Buzzer
  #endif
  digitalWrite(21,LOW);//M5C OFF

  Serial.begin(115200);
  M5.begin();
  Serial.println("M5 begined");

  #ifdef _M5STICKC_H_
  M5.Lcd.setRotation(3);              // 画面の向きを変更（右横向き）Change screen orientation (left landscape orientation).
  M5.Axp.ScreenBreath(9);            // 液晶バックライト電圧設定 LCD backlight voltage setting.
  M5.Lcd.fillScreen(BLACK);           // 画面の塗りつぶし　Screen fill.
  draw_sense_mode(sense_mode);
  #endif

  Serial.println("WiFi Setting...");
  delay(10);
  int a = esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_11B );
  //ESP_ERROR_CHECK( esp_wifi_set_protocol( WIFI_IF_STA, WIFI_PROTOCOL_11B ) ); //Error happend
  wifiMulti.addAP("ESPAsyncWebServer", "");
  wifiMulti.addAP("ESP32", "");
  //https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#esp32-wi-fi-configuration
  //https://gist.github.com/yaqwsx/ac662c9b600ef39a802da0be1b25d32d
  Serial.println("Connecting Wifi...");
  if(wifiMulti.run() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
  }


  //=====Init vl53l0x sensor==========

#if defined(VL53L0X_h)
  //Init vl53l0x
  #ifdef _M5STICKC_H_
  Wire.begin(0, 26, 100000U);
  //Wire.begin(32, 33, 100000); 
  #endif
  #ifdef _M5ATOM_H_
  Wire.begin(26, 32, 100000U);
  #endif

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

  atomecho.InitI2SSpeakerOrMic();
}

void loop() {
  delay(1); 

  if(sense_mode =="start" && M5.Btn.wasReleasefor(2000)){
    sense_mode = "lap";
  }
  else if(sense_mode =="lap" && M5.Btn.wasReleasefor(2000)){
    sense_mode = "start";
  }
  else if(sense_mode =="start" && M5.Btn.wasReleasefor(1000)){
    atomecho.playSound(3);
    digitalWrite(21,LOW); 
    delay(3000); //10秒待機
    
    atomecho.playSound(0); // On your marks
    delay(13000+(rand() % 2)*1000); //13~15秒待機
    atomecho.playSound(1); //  set...
    delay(1300 + (rand() % 5)*100); //1.3~1.8秒待機
    httpGetUltraSonic(sense_mode,true);
    atomecho.playSound(2); // BAN! (pistor)

    #ifdef _M5ATOM_H_
    digitalWrite(21,HIGH);
    delay(100);
    digitalWrite(21,LOW);
    #endif


  }
  else if(M5.Btn.wasReleasefor(20)){
    /*
    atomecho.playSound(0);
    delay(2000);
    atomecho.playSound(1);    
    delay(2000);
    */
    if(TIME_DEBUG) Serial.println("start:"+String(millis()));
    long tmp_start_millis = millis();
    httpGetUltraSonic(sense_mode,true);
    long tmp_stop_millis = millis();
    if(TIME_DEBUG) Serial.println("httpget+sound:"+String(tmp_stop_millis-tmp_start_millis) + "ms");
    if(TIME_DEBUG) Serial.println("stop:"+String(millis()));

    atomecho.playSound(4); 
    //delay(1000);
    //atomecho.playSound(2); //43.5ms
  }
  
  //========WiFi Connection Check========
  currentWiFiMillis = millis();
  if (currentWiFiMillis - previousWiFiMillis >= WiFiCheckInterval) {
    wifiState = wifiMulti.run();
    //Serial.println(wifiState);
    if(wifiState != WL_CONNECTED){
      Serial.println("Reconnecting to WiFi...");
      //M5C_LED_ON_and_OFF(500);
    }else if(wifiState == WL_CONNECTED){
      //Serial.print("WiFi connected.");
      rssi = String(WiFi.RSSI());
      //Serial.println(rssi);
      //draw_payload();
    }
    previousWiFiMillis = currentWiFiMillis;
  }
  
/*
  currentPollingMillis = millis();
  if ((wifiState == WL_CONNECTED)&& (currentPollingMillis - previousPollingMillis >= PollingInterval)) {
     h
     
     ttpGetStatePolling();
     previousPollingMillis = currentPollingMillis;
  }
*/
  //======Sensing and send by http Get ===========
  if (sense_mode == "lap" && !WAV_DEBUG && (wifiState == WL_CONNECTED)&& ((millis() - millisPrevious) > millisPerRead) ) {
    millisPrevious = millis();
      portENTER_CRITICAL_ISR(&mutex);
      #if defined(VL53L0X_h)
        Distance = sensor.readRangeContinuousMillimeters();
      #elif defined(Ultrasonic_H) 
        Distance = ultrasonic.MeasureInCentimeters()*10; // two measurements should keep an interval
      #endif
      portEXIT_CRITICAL_ISR(&mutex);

    //1秒以内の連続http get禁止
    if(((millis() - httpGetExecTime) > 1000) && (distanceThresholdLower < Distance) && (Distance < distanceThreshold)){
      httpGetExecTime = millis();
      measureState = "measuring";
      httpGetUltraSonic(sense_mode,true);
      //atomecho.playSound(2); //43.5ms
      //draw_executed_mode(sense_mode);
      //draw_executed_mode("sense ");
      //draw_payload();
      //M5C_LED_ON_and_OFF(500);
    }
    Serial.print(millis() - millisPrevious);
    Serial.print(",");
    Serial.println(Distance);
  }

#ifdef _M5STICKC_H_
  if(M5.BtnB.wasPressed()){
    httpGetUltraSonic("stop"); 

    //draw_executed_mode("stop  ");
    //draw_payload();
    //M5C_LED_ON_and_OFF(500);
  }else if(M5.BtnA.wasPressed()){//pressed
    httpGetUltraSonic("start");
    //draw_executed_mode("start ");
    //draw_payload();
    //M5C_LED_ON_and_OFF(500);
  }
  
  //====Auto Measure Mode変更====
  if(M5.Axp.GetBtnPress()==1){ //1秒以上押した
    if(sense_mode == "start"){
      sense_mode = "lap";
      //draw_sense_mode(sense_mode);
    }else if sense_mode == "lap"{
      sense_mode = "stop";
      //draw_sense_mode(sense_mode);
    }else{
      sense_mode = "start";
      //draw_sense_mode(sense_mode);
    }
  }
#endif

  M5.update();
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

//Cannot M5.LCD....
//Guru Meditation Error: Core  1 panic'ed (InstrFetchProhibited). Exception was unhandled.
/*
void httpGetStatePolling(){
        HTTPClient http;
        String payload;
        http.begin("http://192.168.4.1/update?output=get_time&state=0&rssi="+String(rssi));
        int httpCode = http.GET(); // Make the request
        if (httpCode > 0) { //Check for the returning code
          payload = http.getString();
          Serial.println(httpCode);
          Serial.println(payload);
        }
        else {
          Serial.println("Error on HTTP request");
          Serial.println(httpCode);
        }
        http.end(); //Free the resources
        M5.Lcd.setCursor(1, 1, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(payload);
}
*/


#ifdef _M5STICKC_H_
void init(){
  digitalWrite(10,HIGH);//M5C OFF
}

void draw_sense_mode(String mode){
          M5.Lcd.setCursor(1, 1, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.print("Auto-Mode: " + mode + "  ");  
}

void draw_executed_mode(String mode){
          M5.Lcd.setCursor(1, 20, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.print("Executed: " + mode + "  ");  
}
void draw_payload(){
          M5.Lcd.setCursor(1, 40, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.print("ReturnData: " + payload + "     ");    
}


void draw_WiFi_State(String state){
          M5.Lcd.setCursor(1, 60, 2);  //https://lang-ship.com/reference/unofficial/M5StickC/Tips/M5Display/
          M5.Lcd.setTextColor(WHITE, BLACK);
          M5.Lcd.print("WiFi: " + state + "     ");    
}

void M5C_LED_ON_and_OFF(int ontime){
    digitalWrite(10,LOW);//M5C on
    delay(ontime);
    digitalWrite(10,HIGH);//M5C off  
}
#endif