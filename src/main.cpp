#define F_CPU 9600000

// #include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define PULLDOWN
#define DEBUG
// #define DEBUGALL

#if defined(DEBUG) || defined(DEBUGALL)
#include "uart.h"
UART uart;
#endif

#define STREET_LIGHT PB4
// #define SENSOR A3
#define SENSOR PB3
#define PERIOD 60 // Колличество измерений (1 измерение = 1-й секунде)



uint8_t lampState = 0;
volatile uint16_t count = 0;

void timer_init();
void adc_init();
uint8_t adc_read();
uint8_t getAvarege(uint8_t period);
void switchStreetLight(uint8_t currentLight);

ISR (TIM0_OVF_vect) {
  count++;
  TCNT0 = 162; // обновляем регистр таймера
  // #ifdef DEBUG
  //   uart.print_P(PSTR("Count: "));
  //   uart.print(count);
  //   uart.println();
  // #endif
}

int main() {
  uint8_t currentLight = 0;
  OSCCAL = 208;
  PORTB |= (1<<STREET_LIGHT);
  // PORTB &= ~(1<<STREET_LIGHT);
  DDRB |= (1<<STREET_LIGHT);
  #if defined(DEBUG) || defined(DEBUGALL)
  uart.begin();
  #endif
  adc_init();
  timer_init();
  sei();

  while (1) {
    if (count >= (PERIOD*1000)) {
      count = 0;
      cli();
      currentLight = getAvarege(PERIOD);
      count = 0;
      #ifdef DEBUG
        uart.print_P(PSTR("Average data: "));
        uart.print(currentLight);
        uart.println();
      #endif
    }
    switchStreetLight(currentLight);
  }
  
  return 0;
}

void adc_init(void) {
  // Set the ADC input to PB3/ADC3, left adjust result
  ADMUX |= (1<<MUX0)|(1<<MUX1)|(1<<ADLAR);
  // Set the prescaler to clock/128 & enable ADC
  ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}

void timer_init() {
  TCCR0B |= (1<<CS02)|(1<<CS00);  /* Делитель 1024, 9600000/1024 = 9375 - частота тактов таймера
                                   * 1/9375 = 107 мкс - длительность одного такта таймера
                                   * 107 * 256 = 27392 мкс = 0,27392 сек. длительность одного прерывания
                                   * 107 * 94 = 10058 мкс ~ 0,1 сек длительность прерывания на 94 тика
                                   */ 
  TIMSK0 |= (1<<TOIE0); // Разрешение прерываний
  TCNT0 = 162; // 8-и битный регистр таймера, 256 - 94 = 162
}

uint8_t adc_read (void) {
  // Start the conversion
  ADCSRA |= (1 << ADSC);
  // Wait for it to finish - blocking
  while (ADCSRA & (1 << ADSC));
  return ADCH;
}

uint8_t getAvarege(uint8_t period) {
  uint16_t avgLight = 0;
  uint8_t curLight = 0;
  uint8_t cycleCount = 0;
  sei();

  while (cycleCount < period)
  {
    if (count > period) {
      cycleCount++;
      curLight = adc_read();
      avgLight += curLight;
      #ifdef DEBUGALL
          uart.print(curLight);
          uart.println();
      #endif
    }
  }
  avgLight = avgLight/period;
  return avgLight;
}

void switchStreetLight(uint8_t currentLight) {
#ifdef PULLDOWN
  if ((currentLight < 50)&&(!lampState))
  {
    #ifdef DEBUG
        uart.print_P(PSTR(" lamp \"On\""));
    #endif
    PORTB |= (1<<STREET_LIGHT);
    // digitalWrite(STREET_LIGHT, HIGH);
    lampState = 1;
  }
  if ((currentLight > 70)&&(lampState))
  {
    #ifdef DEBUG
        uart.print_P(PSTR(" lamp \"Off\""));
    #endif
    // digitalWrite(STREET_LIGHT, LOW);
    PORTB &= ~(1<<STREET_LIGHT);
    lampState = 0;
  }
#endif
#ifndef PULLDOWN
  if ((currentLight > 180)&&(!lampState))
  {
    #ifdef DEBUG
        uart.print_P(PSTR(" lamp \"On\""));
    #endif
    PORTB |= (1<<STREET_LIGHT);
    // digitalWrite(STREET_LIGHT, HIGH);
    lampState = 1;
  }
  if ((currentLight < 160)&&(lampState))
  {
    #ifdef DEBUG
        uart.print_P(PSTR(" lamp \"Off\""));
    #endif
    // digitalWrite(STREET_LIGHT, LOW);
    PORTB &= ~(1<<STREET_LIGHT);
    lampState = 0;
  }
#endif
}