#include <Arduino.h>

#define COOLER PB4
#define SENSOR A3
#define PERIOD 60

#define DEBUG 1
//#define DEBUGALL 1

uint8_t lampState;

void setup() {
  lampState = 0;
  digitalWrite(COOLER, LOW);
  pinMode(COOLER, OUTPUT);
  pinMode(SENSOR, INPUT);
}

void loop() {
  uint32_t lightData = 0;
  uint8_t lightDataCurrent = 0;
  uint8_t count = 0;
#ifdef DEBUG
  Serial.println();
  Serial.write("--");
#endif
  while (count < PERIOD)
  {
    count++;
    lightDataCurrent = 0;
    lightDataCurrent = analogRead(SENSOR);
    lightData += lightDataCurrent;
#ifdef DEBUG
    Serial.print(lightDataCurrent);
    Serial.println();
#endif
    delay(1000);
  }
#ifdef DEBUG
  Serial.write("--");
  Serial.println();
#endif
  lightData = lightData/PERIOD;
#ifdef DEBUGALL
  Serial.write("Average data: ");
  Serial.print(lightData);
  Serial.println();
#endif
#ifdef DEBUG
  Serial.write("Average data: ");
  Serial.print(lightData);
#endif

  if ((lightData > 220)&&(!lampState))
  {
#ifdef DEBUG
    Serial.write(" lamp \"On\"");
#endif
    digitalWrite(COOLER, HIGH);
    lampState = 1;
  }
  if ((lightData < 190)&&(lampState))
  {
#ifdef DEBUG
    Serial.write(" lamp \"Off\"");
#endif
    digitalWrite(COOLER, LOW);
    lampState = 0;
  }
  delay(1000);
}