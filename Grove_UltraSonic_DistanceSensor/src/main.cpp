#include "M5StickC.h"
#include "Ultrasonic.h"

long cnt=0;

Ultrasonic ultrasonic(33);
void setup()
{
    Serial.begin(115200);
}
void loop()
{
    long RangeInInches;
    long RangeInCentimeters;

    /*
    Serial.println("The distance to obstacles in front is: ");
    RangeInInches = ultrasonic.MeasureInInches();
    Serial.print(RangeInInches);//0~157 inches
    Serial.println(" inch");
    delay(250);
    */
    long start = millis();
    RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
    Serial.printf("%d, %d cm ",cnt,RangeInCentimeters); //0~400cm
    long stop = millis();
    Serial.printf("%d ms \n",(stop-start));
    cnt++;
    delay(1);
}