/*******************************************************************************
 ******************************************************************************/
#include <Arduino_GFX_Library.h>

//===GFX setting
#define GFX_BL DF_GFX_BL // default backlight pin, you may replace DF_GFX_BL to actual backlight pin
#define TFT_BL 14
#if defined(DISPLAY_DEV_KIT)
Arduino_GFX *gfx = create_default_Arduino_GFX();
#else /* !defined(DISPLAY_DEV_KIT) */
Arduino_DataBus *bus = new Arduino_ESP32SPI(11 /* DC */, 10 /* CS */, 12 /* SCK */, 13 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 1 /* RST */, 1 /* rotation */, true /* IPS */, 170 /* width */, 320 /* height */, 35 /* col offset 1 */, 0 /* row offset 1 */, 35 /* col offset 2 */, 0 /* row offset 2 */);
#endif


//===WiFi setting===
#define wifiap
#if defined(wifiap)
//#include <esp_wifi.h>
#include <WiFi.h>

extern "C" {
  #include "esp_wifi.h"
}


const char* ssid = "ESPAsyncWebServer";
const char* password = "";
void initWiFi(uint8_t wifi_channel);
void deinitWiFi();
void setWiFiChannel(uint8_t channel);
String printStationRSSI();
uint8_t wifi_channel = 1; //1-13
#endif

#define espnow
#if defined(espnow) and defined(wifiap)
#include <esp_now.h>
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
typedef struct struct_message {
    char a[32];
    int b;
} struct_message;
struct_message myData;
#endif

//===GFX===
TaskHandle_t taskGFXHandle;
#define TASK_SLEEP_GFX 1000
static void GFXLoop(void* arg);
void initGFX();

//===Button===
TaskHandle_t taskButton1Handle;
TaskHandle_t taskButton2Handle;
#define TASK_SLEEP_BUTTON_CHECK 100 //10ms delay
static void Button1CheckLoop(void* arg);
static void Button2CheckLoop(void* arg);


//===Ultrasonic===
#include <Ultrasonic.h>
const int sigPin = 5;  // Pin connected to the SIG pin of the Seeed Ultrasonic Sensor
Ultrasonic ultrasonic(sigPin);  // Create an Ultrasonic object
uint16_t distance_mm,distance_mm_prev,distance_mm_stopped;

TaskHandle_t taskUltrasonicHandle;
#define TASK_SLEEP_ULTRASONIC 15 //20ms delay
static void UltrasonicLoop(void* arg);

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;

//===StopWatch===
TaskHandle_t taskStopWatchHandle;
#define TASK_SLEEP_STOPWATCH 15 //10msa
static void StopWatchLoop(void* arg);
int LED_GPIO = 2;

//=====Webserver===
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";
const char* PARAM_INPUT_3 = "rssi";
AsyncWebServer server(80);
void start_http_server();
String outputState(int output);
String processor(const String& var);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Athletic Timer</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    body {max-width: 150px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 60px; height: 34px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 2px}
    .slider:before {position: absolute; content: ""; height: 10px; width: 10px; left: 2px; bottom: 2px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px}
    input:checked+.slider {background-color: #b30000}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}

    .button {
      background-color: #008CBA;; /* Green */
      border: none;
      color: white;
      padding: 15px 32px;
      text-align: center;
      text-decoration: none;
      display: inline-block;
      font-size: 16px;
　　}
  </style>
</head>
<body>
  <h3>Athletic Timer</h3>
  %BUTTONPLACEHOLDER%
  <font size ="3"><p>State: <span id="measure_state">%MEASURE_STATE% </span></p></font>
  <font size ="3"><p>Lap Time: <span id="lap_time">%LAP_TIME% </span></p></font>
  <font size ="3"><p>Stop Time: <span id="elapsed_time">%ELAPSED_TIME% </span></p></font>
  <font size ="3"><p>Starter Rssi: <span id="rssi"> </span></p></font>
  <font size ="3"><p>Log: <span id="time_log">%TIME_LOG% </span></p></font>
</body>

</html>

<script>
let vl53l0xStopperState_prev="";
setInterval(function ( ) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      let splited = this.responseText.split(",");
      let elapsed_time = splited[0];
      let measure_state = splited[1];
      let vl53l0xStopperState = splited[2];
      let rssi = splited[3];
      let lap_time = splited[4];
      document.getElementById("elapsed_time").innerHTML = elapsed_time;
      document.getElementById("measure_state").innerHTML = measure_state;
      document.getElementById("rssi").innerHTML = rssi;
      document.getElementById("lap_time").innerHTML = lap_time;
      
      
      if( vl53l0xStopperState_prev == "detection_waiting" && vl53l0xStopperState == "detected" ){
        document.getElementById("time_log").innerHTML =  document.getElementById("time_log").innerHTML + "<br>" + "Time:" +  elapsed_time +",Lap:"+ lap_time;
      }
      vl53l0xStopperState_prev = vl53l0xStopperState;
      console.log(vl53l0xStopperState);
    }
  };
  xhttp.open("GET", "/update?output=get_time&state=0", true);
  xhttp.send();
}, 2000 );

function httpGetRequest(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/update?output="+element.id+"&state=1", true); }
  else { xhr.open("GET", "/update?output="+element.id+"&state=0", true); }
  xhr.send();
}

function httpGetStartRequest() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("measure_state").innerHTML = "measuring";
    }
  };
  xhttp.open("GET", "/update?output=start&state=0", true);
  xhttp.send();
}

function httpGetStopRequest() {
  if(document.getElementById("measure_state").innerHTML == "measured"){
    return; //measured状態で行わない。
  }
  
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200){
      let splited = this.responseText.split(",");
      document.getElementById("elapsed_time").innerHTML = splited[0];
      document.getElementById("measure_state").innerHTML = splited[1];
      document.getElementById("lap_time").innerHTML = splited[4];
      document.getElementById("time_log").innerHTML =  document.getElementById("time_log").innerHTML  + "<br>" + "Time:"+ splited[0] +",Lap:"+ splited[4];
    }
  };
  xhttp.open("GET", "/update?output=stop&state=0", true);
  xhttp.send();
}

function httpGetDisplayTime(measureTriggerCause = ''){    
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var splited = this.responseText.split(",");
      document.getElementById("lap_time").innerHTML = splited[4];
      document.getElementById("measure_state").innerHTML = splited[1];
      document.getElementById("time_log").innerHTML =  document.getElementById("time_log").innerHTML + "<br>" + String(measureTriggerCause) +":"+ splited[5] ;
    }
  };
  xhttp.open("GET", "/update?output=get_time&state=0", true);
  xhttp.send();
}

</script>

)rawliteral";


#if defined(m5cp)
void LCD_state_init();
void LCD_state_stop();
void LCD_state_start();
void LCD_state_lap();
#endif

String measureState= "init";
String measureStatePrev ="";
String vl53l0xStarterState= "init";
String vl53l0xStopperState= "init";
unsigned long elapsed_time_ms = 0;
float lap_time_ms = 0;
String rssi;

boolean DEBUG_DISTANCE = false;
boolean HTTP_TIME_DEBUG = true;



//===============================

void setup(void)
{
    Serial.begin(115200);
   
    initGFX();

    //xTaskCreatePinnedToCore(WiFiCheckLoop, TASK_NAME_WIFI, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
    xTaskCreatePinnedToCore(Button1CheckLoop, "ButtonCheckLoop", 4096, NULL, 1, &taskButton1Handle, 1);
    xTaskCreatePinnedToCore(Button2CheckLoop, "ButtonCheckLoop", 4096, NULL, 1, &taskButton2Handle, 1);
    xTaskCreatePinnedToCore(UltrasonicLoop, "UltrasonicLoop", 4096, NULL, 1, &taskUltrasonicHandle, 1);
    xTaskCreatePinnedToCore(GFXLoop, "GFXLoop", 4096, NULL, 1, &taskGFXHandle, 1);
    //start_http_server();
    xTaskCreatePinnedToCore(StopWatchLoop, "StopWatchLoop", 4096, NULL, 1, &taskStopWatchHandle, 1);

    delay(5000); // 5 seconds

    #if defined(wifiap)
    initWiFi(wifi_channel);
    #endif

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
      String get_param1;
      String get_param2;
      
      // GET input1 value on <ESP_IP>/update?output=<get_param1>&state=<get_param2>
      if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
        get_param1 = request->getParam(PARAM_INPUT_1)->value();// equals 2,4,33,start,stop,reset
        get_param2 = request->getParam(PARAM_INPUT_2)->value();// equa0,1
        digitalWrite(get_param1.toInt(), get_param2.toInt());//★指定PinのGPIOをON-OFF

        if(get_param1=="start"){
          request->send(200, "text/plain", "OK");
          measureState = "measuring";
          digitalWrite(LED_GPIO, HIGH); //M5C ON // ESP32 OFF
        }
        else if(get_param1=="vl53l0x_start"){
          digitalWrite(LED_GPIO, HIGH); //M5C OFF
          request->send(200, "text/plain", "start");
          //delay(110);//stopperがhttpgetを受信してから、starterの音声出力までの遅延が約150msのため測定開始を遅らせる
          delay(6);//11b 1Mbps
          measureState = "measuring";
          vl53l0xStarterState = "detected";
          vl53l0xStopperState = "detection_waiting";
          long httpget_rx_time = millis();
          rssi = request->getParam(PARAM_INPUT_3)->value();
          if(HTTP_TIME_DEBUG) Serial.println("start:"+String(millis()));
          long httpget_return_time = millis();
          if(HTTP_TIME_DEBUG) Serial.println("http-process-time:"+String(httpget_return_time - httpget_rx_time));
          if(HTTP_TIME_DEBUG) Serial.println("stop:"+String(millis()));
          //digitalWrite(LED_GPIO, LOW); //M5C ON // ESP32 OFF //timestartと同時点灯させる
          digitalWrite(LED_GPIO, LOW);//M5C ON
        }
        else if(get_param1=="vl53l0x_stop"){
          measureState = "measured";
          vl53l0xStarterState = "init";
          vl53l0xStopperState = "init";
          rssi = request->getParam(PARAM_INPUT_3)->value();
          //digitalWrite(LED_GPIO, HIGH); //M5C ON // ESP32 OFF //timestopと同時に消灯させる
          request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str());
          //delay(20); //wait until lasor sensing.　
        }
        else if(get_param1=="stop"){
          measureState = "measured";
          vl53l0xStarterState = "init";
          vl53l0xStopperState = "init";
          request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str()+String(",")+measureState+String(",")+vl53l0xStopperState+String(",")+rssi.c_str()+String(",")+String(lap_time_ms/1000.0).c_str()); //http response
          digitalWrite(LED_GPIO, HIGH);   
        }
        else if(get_param1=="reset"){
          measureState = "init";
          vl53l0xStarterState = "init";
          vl53l0xStopperState = "init";
          request->send(200, "text/plain", "OK");
        }
        else if(get_param1=="get_lap" || get_param1=="vl53l0x_lap"){
          lap_time_ms = elapsed_time_ms;
          digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
          rssi = request->getParam(PARAM_INPUT_3)->value();
          request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str()+String(",")+measureState+String(",")+vl53l0xStopperState+String(",")+rssi.c_str()+String(",")+String(lap_time_ms/1000.0).c_str()); //http response
          #if defined(m5cp)
          LCD_state_lap();
          #endif
          delay(50);
          digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
        }
        else if(get_param1=="get_time"){
          request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str()+String(",")+measureState+String(",")+vl53l0xStopperState+String(",")+rssi.c_str()+String(",")+String(lap_time_ms/1000.0).c_str()); //http response
          Serial.print(rssi);
          //https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-get-2
        }
        else if(get_param1=="led_onoff"){
          digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
          delay(50);
          digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
          //digitalWrite(LED_GPIO, HIGH); //M5C OFF
          //delay(100);
          //digitalWrite(LED_GPIO, LOW); //M5C OFF
          request->send(200, "text/plain", "OK");
          Serial.print("led_onoff");
          //https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-get-2
        }
        /*
        else if(get_param1=="rssi"){
          rssi = String(get_param2);
          request->send(200, "text/plain", rssi.c_str()); //http response
          //Use this and get 
          //https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-get-2
        }*/
        else{
          request->send(200, "text/plain", "OK");
        }
      }
      else {
        get_param1 = "No message sent";
        get_param2 = "No message sent";
        request->send(200, "text/plain", "No message sent");
      }
      Serial.print("get_param1:");
      Serial.print(get_param1);
      Serial.print("get_param2:");
      Serial.println(get_param2);
    });
    
    server.begin(); //web server start
    //----------------------------------------------------------------------

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
  //startMillis = millis();    
}

void loop()
{
  delay(100);
}


//----------WiFI------
void initWiFi(uint8_t wifi_channel){
  WiFi.mode(WIFI_AP);//for AP mode
    //here config LR mode
  int a= esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_11B );
  int b = esp_wifi_config_80211_tx_rate(WIFI_IF_AP,WIFI_PHY_RATE_1M_L);
  int c = esp_wifi_set_max_tx_power(80);//20dBm https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html

  WiFi.softAP(ssid, password,wifi_channel);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error  initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  
  /*IPAddress Ip(192, 168, 3, 1);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);
  */
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");Serial.println(myIP);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");

  Serial.println(millis());
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");

  gfx->setTextSize(1);
  gfx->setCursor(240, 130); gfx->println("Rx:           .");
  gfx->setCursor(240, 130); gfx->println("Rx:"+String(myData.a));
}



void setWiFiChannel(uint8_t channel){  
  deinitWiFi();
  initWiFi(channel);

  gfx->setTextSize(1);
  gfx->fillRect(240,130,100,10,BLACK);
  gfx->setCursor(240, 130); gfx->println("Channel:"+String(channel));
}


void deinitWiFi(){
  esp_now_deinit();
  WiFi.softAPdisconnect();
  WiFi.disconnect(); //WiFI module OFF.
}

String getStationRSSIasString() {
  wifi_sta_list_t stationList;
  String rssiList = "";

  if (esp_wifi_ap_get_sta_list(&stationList) == ESP_OK) {
    for (int i = 0; i < stationList.num; i++) {
      wifi_sta_info_t station = stationList.sta[i];

      // RSSIを取得
      wifi_ap_record_t apRecord;
      if (esp_wifi_sta_get_ap_info(&apRecord) == ESP_OK) {
        rssiList += "Station " + String(i + 1) + ": RSSI: " + String(apRecord.rssi);
        if (i < stationList.num - 1) {
          rssiList += ", ";
        }
      }
    }
  }
  return rssiList;
}


//---------------

void initGFX(){
    gfx->begin();
    gfx->fillScreen(BLACK);
    gfx->setTextSize(2);

    #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    //digitalWrite(TFT_BL, HIGH);
    analogWrite(TFT_BL, 5); //一定時間操作がなければOFFが良い
    #endif

    gfx->setTextColor(WHITE);
    gfx->setCursor(10, 10); gfx->println("Stop-Time:");
    gfx->setCursor(10, 30); gfx->println("Lap -Time:");
    gfx->setCursor(10, 50); gfx->println("Distance :");
    gfx->setCursor(10, 70); gfx->println("Stop-Dist:");
    gfx->setCursor(10, 90); gfx->println("State    :");
    gfx->setCursor(10, 110); gfx->println("RSSI-stater:");
    gfx->setCursor(10, 130); gfx->println("RSSI-lapper:");

    gfx->setTextSize(1);
    gfx->setCursor(240, 140); gfx->println("Channel:"+String(wifi_channel));
    gfx->setCursor(240, 150); gfx->println("Track&Field");
    gfx->setTextSize(2);
}



static void GFXLoop(void* arg){
  //Detail is here. https://github.com/moononournation/Arduino_GFX
  while (1) {
    uint32_t entryTime = millis();
  
    static uint16_t initx = 170;
    gfx->setTextSize(2);
    gfx->setCursor(initx, 10); gfx->fillRect(initx,10,100,20,BLACK); gfx->println(String(elapsed_time_ms));
    gfx->setCursor(initx, 30); gfx->fillRect(initx,30,100,20,BLACK); gfx->println(2);
    gfx->setCursor(initx, 50); gfx->fillRect(initx,50,100,20,BLACK); gfx->println(String(distance_mm));
    gfx->setCursor(initx, 70); gfx->fillRect(initx,70,100,20,BLACK); gfx->println(String(distance_mm_stopped));
    gfx->setCursor(initx, 90); gfx->fillRect(initx,90,100,20,BLACK); gfx->println(measureState);
    gfx->setCursor(initx, 110); gfx->fillRect(initx,110,100,20,BLACK); gfx->println(getStationRSSIasString());
    gfx->setCursor(initx, 130); gfx->fillRect(initx,130,100,20,BLACK); gfx->println(7);

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_GFX  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}



static void UltrasonicLoop(void* arg){

  while (1) {
    uint32_t entryTime = millis();

    portENTER_CRITICAL_ISR(&mutex);
    distance_mm = ultrasonic.MeasureInCentimeters((TASK_SLEEP_ULTRASONIC-0.2)*1000)*10;
    portEXIT_CRITICAL_ISR(&mutex);
    //Serial.println(distance_mm);


    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_ULTRASONIC  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}






static void Button1CheckLoop(void* arg){
  static boolean button_state1 = false;
  static boolean prev_button_state1 = false;
  const uint8_t pin_button1 = 47;
  pinMode(pin_button1,INPUT_PULLUP);
  pinMode(45,OUTPUT);
  digitalWrite(45,LOW);
  
  while (1) {
    uint32_t entryTime = millis();

    //==button1==
    prev_button_state1 = button_state1;
    button_state1 = digitalRead(pin_button1);
    if(button_state1 == false && prev_button_state1 == true){
      Serial.print("Pushed1");
      //vTaskSuspend(taskUltrasonicHandle);
    }

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_BUTTON_CHECK  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}


static void Button2CheckLoop(void* arg){
  static boolean button_state2 = false;
  static boolean prev_button_state2 = false;
  const uint8_t pin_button2 = 40;
  pinMode(pin_button2,INPUT_PULLUP);
  pinMode(38,OUTPUT);
  digitalWrite(38,LOW);
  
  while (1) {
    uint32_t entryTime = millis();

    //==button2==
    prev_button_state2 = button_state2;
    button_state2 = digitalRead(pin_button2);  
    if(button_state2 == false && prev_button_state2 == true){
      Serial.print("Pushed2");
      //vTaskResume(taskUltrasonicHandle);
      wifi_channel = (wifi_channel+1)%13;
      if(wifi_channel==0) wifi_channel=1;
      setWiFiChannel(wifi_channel);
    }

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_BUTTON_CHECK  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}





// Replaces placeholder with button section in your web page
String processor(const String& var){
  //Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons = "";
    buttons += "<h4>LED</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"httpGetRequest(this)\" id=\"10\" " + outputState(10) + "><span class=\"slider\"></span></label>";
    buttons += "<h4></h4><button type=\"button\" onclick=\"httpGetStartRequest()\" id=\"start\" class=\"button\"> start </button>";
    buttons += "<h4></h4><button type=\"button\" onclick=\"httpGetStopRequest()\" id=\"stop\" class=\"button\"> stop </button>";
    buttons += "<h4></h4><button type=\"button\" onclick=\"httpGetDisplayTime('lap')\" id=\"get_lap\" class=\"button\"> Lap </button>";
    //buttons += "<h4></h4><button type=\"button\" onclick=\"httpGetDisplayTime()\" id=\"get_time\" class=\"button\"> GetTime </button>";
    return buttons;
  }
  else if(var == "MEASURE_STATE"){
    return measureState;
  }
  else if(var == "ELAPSED_TIME"){
    return String(elapsed_time_ms/1000.0);
  }
  else if(var == "TIME_LOG"){
    return String("");
  }
  return String();
}

String outputState(int output){  
  if(digitalRead(output)){
    return "checked";
  }
  else {
    return "";
  }
}


  //--------Waiting http Access---------
void start_http_server(){
//=====Web Server Setting=====


  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String get_param1;
    String get_param2;
    
    // GET input1 value on <ESP_IP>/update?output=<get_param1>&state=<get_param2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      get_param1 = request->getParam(PARAM_INPUT_1)->value();// equals 2,4,33,start,stop,reset
      get_param2 = request->getParam(PARAM_INPUT_2)->value();// equa0,1
      digitalWrite(get_param1.toInt(), get_param2.toInt());//★指定PinのGPIOをON-OFF

      if(get_param1=="start"){
        request->send(200, "text/plain", "OK");
        measureState = "measuring";
        digitalWrite(LED_GPIO, HIGH); //M5C ON // ESP32 OFF
      }
      else if(get_param1=="vl53l0x_start"){
        digitalWrite(LED_GPIO, HIGH); //M5C OFF
        request->send(200, "text/plain", "start");
        //delay(110);//stopperがhttpgetを受信してから、starterの音声出力までの遅延が約150msのため測定開始を遅らせる
        delay(6);//11b 1Mbps
        measureState = "measuring";
        vl53l0xStarterState = "detected";
        vl53l0xStopperState = "detection_waiting";
        long httpget_rx_time = millis();
        rssi = request->getParam(PARAM_INPUT_3)->value();
        if(HTTP_TIME_DEBUG) Serial.println("start:"+String(millis()));
        long httpget_return_time = millis();
        if(HTTP_TIME_DEBUG) Serial.println("http-process-time:"+String(httpget_return_time - httpget_rx_time));
        if(HTTP_TIME_DEBUG) Serial.println("stop:"+String(millis()));
        //digitalWrite(LED_GPIO, LOW); //M5C ON // ESP32 OFF //timestartと同時点灯させる
        digitalWrite(LED_GPIO, LOW);//M5C ON
      }
      else if(get_param1=="vl53l0x_stop"){
        measureState = "measured";
        vl53l0xStarterState = "init";
        vl53l0xStopperState = "init";
        rssi = request->getParam(PARAM_INPUT_3)->value();
        //digitalWrite(LED_GPIO, HIGH); //M5C ON // ESP32 OFF //timestopと同時に消灯させる
        request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str());
        //delay(20); //wait until lasor sensing.　
      }
      else if(get_param1=="stop"){
        measureState = "measured";
        vl53l0xStarterState = "init";
        vl53l0xStopperState = "init";
        request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str()+String(",")+measureState+String(",")+vl53l0xStopperState+String(",")+rssi.c_str()+String(",")+String(lap_time_ms/1000.0).c_str()); //http response
        digitalWrite(LED_GPIO, HIGH);   
      }
      else if(get_param1=="reset"){
        measureState = "init";
        vl53l0xStarterState = "init";
        vl53l0xStopperState = "init";
        request->send(200, "text/plain", "OK");
      }
      else if(get_param1=="get_lap" || get_param1=="vl53l0x_lap"){
        lap_time_ms = elapsed_time_ms;
        digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
        rssi = request->getParam(PARAM_INPUT_3)->value();
        request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str()+String(",")+measureState+String(",")+vl53l0xStopperState+String(",")+rssi.c_str()+String(",")+String(lap_time_ms/1000.0).c_str()); //http response
        #if defined(m5cp)
        LCD_state_lap();
        #endif
        delay(50);
        digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
      }
      else if(get_param1=="get_time"){
        request->send(200, "text/plain", String(elapsed_time_ms/1000.0).c_str()+String(",")+measureState+String(",")+vl53l0xStopperState+String(",")+rssi.c_str()+String(",")+String(lap_time_ms/1000.0).c_str()); //http response
        Serial.print(rssi);
        //https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-get-2
      }
      else if(get_param1=="led_onoff"){
        digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
        delay(50);
        digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
        //digitalWrite(LED_GPIO, HIGH); //M5C OFF
        //delay(100);
        //digitalWrite(LED_GPIO, LOW); //M5C OFF
        request->send(200, "text/plain", "OK");
        Serial.print("led_onoff");
        //https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-get-2
      }
      /*
      else if(get_param1=="rssi"){
        rssi = String(get_param2);
        request->send(200, "text/plain", rssi.c_str()); //http response
        //Use this and get 
        //https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-get-2
      }*/
      else{
        request->send(200, "text/plain", "OK");
      }
    }
    else {
      get_param1 = "No message sent";
      get_param2 = "No message sent";
      request->send(200, "text/plain", "No message sent");
    }
    Serial.print("get_param1:");
    Serial.print(get_param1);
    Serial.print("get_param2:");
    Serial.println(get_param2);
  });
  
  server.begin(); //web server start
  //----------------------------------------------------------------------

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
//startMillis = millis();
}



static void StopWatchLoop(void* arg){

  pinMode(41,OUTPUT);
  digitalWrite(41,LOW);


  long millisPrevious_Distance_dbg = 0;
  //int Distance_prev = 0;
  long Distance_dbg_intervalms = 2000;

  int Duration;
  //uint16_t Distance;
  unsigned long startMillis,stopMillis,millisPrevious=0;
  boolean DEBUG_vl53l0x = false;  //Serial Load is Heavy. If ON, sometimes unknown error happend. unknown character were printed.

  #if defined(VL53L0X_h)
    int distanceThreshold = 1000; //mm
    int distanceThresholdLower = 200; //mm
    unsigned long millisPerRead = 20;
  #elif defined(Ultrasonic_H)
    unsigned long millisPerRead = 10;
    int distanceThreshold = 1000; //mm
    int distanceThresholdLower = 5; //mm  
  #endif


  while(1){
    uint32_t entryTime = millis();

  //if((millis() - millisPrevious) > millisPerRead){
    //===Measure Ultra Sonic===
    //delay(5000); //初回スタート時間は5秒経過以降
    //if(millisPrevious > 5000){
      //===HTTP Get Check===
      //start received by http-get
      if ((measureStatePrev=="init"||measureStatePrev=="measured") && measureState=="measuring") {
          startMillis = millis();
          lap_time_ms = 0;
          digitalWrite(LED_GPIO, LOW);
          #if defined(m5cp)
          LCD_state_start();
          #endif
      }

      //===Distance Check===
      /*
      if (measureState=="measuring" || (millis() - millisPrevious_Distance_dbg) > Distance_dbg_intervalms ) {
        portENTER_CRITICAL_ISR(&mutex);
        #if defined(VL53L0X_h)
          Distance = sensor.readRangeContinuousMillimeters();
        #elif defined(Ultrasonic_H)       
          Distance = ultrasonic.MeasureInCentimeters()*10; // two measurements should keep an interval
        #endif
        portEXIT_CRITICAL_ISR(&mutex);
      }*/


      if (distance_mm>0) {
       if(DEBUG_vl53l0x){
         Serial.print(distance_mm);
         Serial.println(" mm");
       }
      }
    
      //Ultra Sonic Debug
      if(DEBUG_DISTANCE){
        Serial.print(millis() - millisPrevious);
        Serial.print(",");
        Serial.println(distance_mm);
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
          digitalWrite(LED_GPIO, HIGH);  
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
            Serial.print(distance_mm);
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
      else if(abs(distance_mm - distance_mm_prev) > 200){
        if(measureState=="measuring"){
          //stop timer
          stopMillis = millis();
          elapsed_time_ms = stopMillis - startMillis;
          digitalWrite(LED_GPIO, HIGH); 
          distance_mm_stopped = distance_mm;
          #if defined(m5cp)
          LCD_state_stop();
          #endif

          if(DEBUG_vl53l0x){
            Serial.print(startMillis);
            Serial.print(",");
            Serial.println(stopMillis);
            Serial.print(",");
            Serial.print(String(stopMillis - startMillis) + "ms,");
            Serial.print(distance_mm);
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
    
      measureStatePrev = measureState;
      distance_mm_prev = distance_mm; 

    //millisPrevious = millis();
  //}
  uint32_t elapsed_time = millis() - entryTime; 
  int32_t sleep = TASK_SLEEP_STOPWATCH  - elapsed_time;
  vTaskDelay((sleep > 0) ? sleep : 0);
  }
}
