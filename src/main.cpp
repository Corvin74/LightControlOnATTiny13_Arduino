#include <Arduino.h>

#define COOLER PB4
#define SENSOR A3

void setup() {
  // Serial.begin(9600);
  digitalWrite(COOLER, LOW);
  pinMode(COOLER, OUTPUT);
  pinMode(SENSOR, INPUT);
}

void loop() {
  uint16_t lightData = 0;
  uint8_t lightDataCurrent = 0;
  uint8_t count = 0;
  //Serial.write("\r\n");
  Serial.println();

  while (count < 10)
  {
    // Serial.print("Data from sensor iteration ");
    // Serial.print(count);
    // Serial.print(": ");
    count++;
    lightDataCurrent = 0;
    lightDataCurrent = analogRead(SENSOR);
    lightData += lightDataCurrent;
    // Serial.print(lightDataCurrent);
    // Serial.print("\n\r");
    delay(2000);
  }
  lightData = lightData/10;
  // lightData = analogRead(SENSOR);
  Serial.write("Average data from sensor: ");

  if (lightData > 210)
  {
    Serial.print(lightData);
    Serial.write(" lamp \"On\"");
    digitalWrite(COOLER, HIGH);
  }
  if (lightData < 190)
  {
    Serial.print(lightData);
    Serial.write(" lamp \"Off\"");
    digitalWrite(COOLER, LOW);
  }
  delay(5000);
}