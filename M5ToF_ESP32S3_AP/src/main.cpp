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
#include <esp_wifi.h>
#include <WiFi.h>
const char* ssid = "ESPAsyncWebServer";
const char* password = "";
void initWiFi(uint8_t wifi_channel);
void deinitWiFi();
void setWiFiChannel(uint8_t channel);
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
#define TASK_SLEEP_GFX 200
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
float distance_mm;

TaskHandle_t taskUltrasonicHandle;
#define TASK_SLEEP_ULTRASONIC 100 //10ms delay
static void UltrasonicLoop(void* arg);

portMUX_TYPE mutex = portMUX_INITIALIZER_UNLOCKED;




void setup(void)
{
    Serial.begin(115200);
   
    initGFX();

    //xTaskCreatePinnedToCore(WiFiCheckLoop, TASK_NAME_WIFI, TASK_STACK_DEPTH, NULL, 1, NULL, TASK_DEFAULT_CORE_ID);
    xTaskCreatePinnedToCore(Button1CheckLoop, "ButtonCheckLoop", 4096, NULL, 1, &taskButton1Handle, 1);
    xTaskCreatePinnedToCore(Button2CheckLoop, "ButtonCheckLoop", 4096, NULL, 1, &taskButton2Handle, 1);
    //xTaskCreatePinnedToCore(UltrasonicLoop, "UltrasonicLoop", 4096, NULL, 1, &taskUltrasonicHandle, 1);
    xTaskCreatePinnedToCore(GFXLoop, "GFXLoop", 4096, NULL, 1, &taskGFXHandle, 1);

    delay(5000); // 5 seconds

    #if defined(wifiap)
    initWiFi(wifi_channel);
    #endif
}

void loop()
{
  delay(100);
}

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

void initGFX(){
    gfx->begin();
    gfx->fillScreen(BLACK);
    gfx->setTextSize(2);

    #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    //digitalWrite(TFT_BL, HIGH);
    analogWrite(TFT_BL, 30);
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
    gfx->setCursor(initx, 10); gfx->println(1);
    gfx->setCursor(initx, 30); gfx->println(2);
    gfx->setCursor(initx, 50); gfx->println(3);
    gfx->setCursor(initx, 70); gfx->println(4);
    gfx->setCursor(initx, 90); gfx->println(5);
    gfx->setCursor(initx, 110); gfx->println(6);
    gfx->setCursor(initx, 130); gfx->println(7);

    uint32_t elapsed_time = millis() - entryTime; 
    int32_t sleep = TASK_SLEEP_GFX  - elapsed_time;
    vTaskDelay((sleep > 0) ? sleep : 0);
  }
}



static void UltrasonicLoop(void* arg){

  while (1) {
    uint32_t entryTime = millis();

    portENTER_CRITICAL_ISR(&mutex);
    distance_mm = ultrasonic.MeasureInCentimeters()*10;
    portEXIT_CRITICAL_ISR(&mutex);
    Serial.println(distance_mm);

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
  
  while (1) {
    uint32_t entryTime = millis();

    //==button1==
    prev_button_state1 = button_state1;
    button_state1 = digitalRead(47);
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
  const uint8_t pin_button2 = 48;
  pinMode(pin_button2,INPUT_PULLUP);
  pinMode(38,OUTPUT);
  digitalWrite(38,LOW);
  
  while (1) {
    uint32_t entryTime = millis();

    //==button2==
    prev_button_state2 = button_state2;
    button_state2 = digitalRead(48);  
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