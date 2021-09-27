#include <Arduino.h>
#include "uart.h"

#define STREET_LIGHT PB4
#define SENSOR A3
#define PERIOD 60

UART uart;

#define DEBUG
// #define DEBUGALL 1

uint8_t lampState;

void setup() {
  OSCCAL = 208;
  lampState = 0;
  digitalWrite(STREET_LIGHT, LOW);
  pinMode(STREET_LIGHT, OUTPUT);
  pinMode(SENSOR, INPUT);
  uart.begin();
}

void loop() {
  uint32_t lightData = 0;
  uint8_t lightDataCurrent = 0;
  uint8_t count = 0;
#ifdef DEBUG
  uart.println();
  uart.print_P(PSTR("--"));
#endif
  while (count < PERIOD)
  {
    count++;
    lightDataCurrent = 0;
    lightDataCurrent = analogRead(SENSOR);
    lightData += lightDataCurrent;
#ifdef DEBUG
    uart.print(lightDataCurrent);
    uart.println();
#endif
    delay(1000);
  }
#ifdef DEBUG
  uart.print_P(PSTR("--"));
  uart.println();
#endif
  lightData = lightData/PERIOD;
#ifdef DEBUGALL
  uart.write("Average data: ");
  uart.print(lightData);
  uart.println();
#endif
#ifdef DEBUG
  uart.print_P(PSTR("Average data: "));
  uart.print(lightData);
#endif

  if ((lightData > 170)&&(!lampState))
  {
#ifdef DEBUG
    uart.print_P(PSTR(" lamp \"On\""));
#endif
    digitalWrite(STREET_LIGHT, HIGH);
    lampState = 1;
  }
  if ((lightData < 160)&&(lampState))
  {
#ifdef DEBUG
    uart.print_P(PSTR(" lamp \"Off\""));
#endif
    digitalWrite(STREET_LIGHT, LOW);
    lampState = 0;
  }
  delay(1000);
}