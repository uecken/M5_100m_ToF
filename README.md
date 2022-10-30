# M5_100m_ToF

https://protopedia.net/prototype/2356

# System 

 With in 50 meter, WiFi Repeater is not needed.
 Starter selects better WiFi AP to communicate with Stopper.
![M5C ToF (5)](https://user-images.githubusercontent.com/5786149/142982401-c69bb932-08fa-433e-9b68-e33421ef421e.jpg)




# How to compile

On PlatformIO, open the workspace file.  
Write the ToF Client to M5 Atom Echo.  
Write the ToF AP to M5 Atom Echo.  

# Sensor  
https://www.seeedstudio.com/Grove-Ultrasonic-Distance-Sensor.html![M5C ToF システム構成図](https://user-images.githubusercontent.com/5786149/142982037-6bcaad4c-5f5a-4f8e-b08d-79fbb1261ddd.jpg)


# Update history Memo
- 2022/10/30
  - M5ATOM/M5StickCのG0ピンをLOWに落とさないとWiFi送信電力が低下する問題の対応 
  - M5StampのG10と5Vを1KΩ抵抗でショートすると起動時にLAPモードで起動するように変更
  - 河川敷でM5CとM5Stampで80mの通信確認.ESP32C3の外部アンテナが必要か.
  - TP4056、Sharp製ソーラーパネル、CR123A電池でM5StampC3U起動確認.
- 2022/10/16
  - 親機から子機のHTTP返答が遅くなる、または子機のHTTP受信が遅くなる
    - → 15回目の送信以降で遅くなる
    - → ☆Serial.available()かどうか、全て判定しないと遅くなる
  - 90mで-92dBmとぎりぎり
  - Start状態で、Start押されたときAPのLEDをON/OFFさせたい　→ LAPモードにして確認すればよい
  - M5Stampの消費電力が少なすぎて、バッテリーからの供給電源が落ちてしまう。リチウム3本にするか？
