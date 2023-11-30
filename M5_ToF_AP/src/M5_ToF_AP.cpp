
/***
Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-async-web-server-espasyncwebserver-library/
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

//#include <M5StickC.h>

//#define m5cp
#define esp32s3
#if defined (m5cp)
#include <M5StickCPlus.h>
#elif defined(esp32s3)
#include <Wire.h>
#endif


/*#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
*/
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <esp_wifi.h>

//#include <VL53L0X.h>
#include "Ultrasonic.h"
#if defined(VL53L0X_h)
  VL53L0X sensor; 
  TFT_eSprite img = TFT_eSprite(&M5.Lcd); 
#elif defined(Ultrasonic_H)
  Ultrasonic ultrasonic(33);
#endif

#include <Ticker.h>
Ticker blinker;

const char* ssid = "ESPAsyncWebServer";
const char* password = "";


//=====Web Server Setting=====
const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";
const char* PARAM_INPUT_3 = "rssi";
AsyncWebServer server(80);
String outputState(int output);

#if defined(m5cp)
void LCD_state_init();
void LCD_state_stop();
void LCD_state_start();
void LCD_state_lap();
#endif
void LED_ONOFF();
void LED_ONOFF_Reverse(int LED_GPIO);


String measureState= "init";
String measureStatePrev ="";
String vl53l0xStarterState= "init";
String vl53l0xStopperState= "init";
unsigned long elapsed_time_ms = 0;
float lap_time_ms = 0;
String rssi;

int LED_GPIO = 10;
int LED_GPIO2 = 26;
boolean DEBUG_DISTANCE = true;
boolean HTTP_TIME_DEBUG = true;

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
//====End Web Server Setting=====


//=====http client=====
String httpGETRequest(const char* serverName) {
  HTTPClient http;
  http.begin(serverName);
  int httpResponseCode = http.GET();
  String payload = "{}"; 
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  http.end();  // Free resources
  return payload;
}
//=====(End) http client=====


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
  #endif

  Serial.begin(115200);

  //WiFi感度劣化対策　https://twitter.com/uecken/likes
  //GPIO LOW出力で改善
  pinMode(0,OUTPUT);
  digitalWrite(0,LOW);

  
  //-----Digital Out for Debug-----
  pinMode(LED_GPIO, OUTPUT);
  digitalWrite(LED_GPIO, HIGH); //M5は初期値High、ESP32は初期値Low
  //-----softAP Setting-----
  WiFi.mode( WIFI_AP );//for AP mode
    //here config LR mode
  int a= esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_11B );
  int b = esp_wifi_config_80211_tx_rate(WIFI_IF_AP,WIFI_PHY_RATE_1M_L);
  int c = esp_wifi_set_max_tx_power(80);//20dBm https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html


  WiFi.softAP(ssid, password);
  
  /*IPAddress Ip(192, 168, 3, 1);
  IPAddress NMask(255, 255, 255, 0);
  WiFi.softAPConfig(Ip, Ip, NMask);
  */
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");Serial.println(myIP);
  
  //--------Waiting http Access---------
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
        LED_ONOFF_Reverse(LED_GPIO2);
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

void LED_ONOFF(){
  digitalWrite(LED_GPIO, !digitalRead(LED_GPIO));
}

void LED_ONOFF_Reverse(int LED_GPIO2){
    pinMode(LED_GPIO2, OUTPUT);
    digitalWrite(LED_GPIO2, HIGH);
    delay(200);
    digitalWrite(LED_GPIO2, LOW);
}


long millisPrevious_Distance_dbg = 0;
int Distance_prev = 0;
long Distance_dbg_intervalms = 2000;
void loop() {
  delay(1);
  if((millis() - millisPrevious) > millisPerRead){
    //===Measure Ultra Sonic===
    
    if(millisPrevious > 5000){ //初回スタート時間は5秒経過以降
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
          digitalWrite(LED_GPIO, HIGH); 
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
