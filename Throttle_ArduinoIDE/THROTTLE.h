#ifndef _THROTTLE_H_
#define _THROTTLE_H_

#include <Arduino.h>

/********************************************************************
Date: 08/01/2023

@brief
  Throttle sensor send back the voltage value from 0V -> 5V.

  The analog reading of Arduino receive the value from 199 -> 903

  But when we starts the throttle, after the first time, the analog value
  will stay above 240 (in my experience), so i will map the analog reading value 
  from 250 -> 903 for 0% -> 100% openning throttle

Formula: 
  measured_pos = (((float) voltage_signal - 250) / (903 - 250)) * 100;

H-Bridge GPIO:
  IN1: B0
  IN2: B3
  PWM: B1

Throttle wiring:
  1. Motor positive           (White)
  2. Motor negative           (Black)
  3. Sensor negative          (Yellow)
  4. Sensor ADC reading #1    (Red and black)
  5. Sensor positive          (Green)
  6. Sensor ADC reading #2    (Brown)

NOTE: Sensor ADC reading #1 has only two values, 2.5V at 0% openning and 5V at 100% openning,
so i don't use that sensor for controlling the throttle!
**********************************************************************/
#define F_ARDUINO       16000000

#define MOTOR_DDR       DDRB
#define MOTOR_PORT      PORTB
#define MOTOR_POS       3
#define MOTOR_PWM       1
#define MOTOR_NEG       0

#define MAX_ANALOG_READ  903 
#define MIN_ANALOG_READ  250 

#define Sampling_time       20          // ms
#define inv_Sampling_time   50          // 1/Sampling_time
#define PWM_Period          20000       // 20000 cycles = 1ms; F = 16MHz

void handleSerialCommand(char command);

void TIMER1_INIT()
{
  //Using timer1 for PWM generator, FAST PWM mode 14
  TCCR1A = (1<<WGM11) | (1<<COM1A1) | (1<<COM1B1);
  TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10);
}

void SENSOR_ADC_READING_INIT()
{
  ADMUX &= ~((1<<REFS0) | (1<<REFS1));  // using default voltage reference of arduino
  ADMUX &= ~(15<<0);    // choose A0 as input channel
  ADCSRA |= (1<<ADEN);  // ADC enable
}

void TIMER2_INIT()
{
  // TIMER 2 initialization
  // For sampling time, 16ms, prescaler 1024;
  TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);
  TCNT2 = 5;
  TIMSK2 = (1<<TOIE2);
}

void MOTOR_INIT()
{
  DDRD = 0x00;
  PORTD = 0xFF;

  MOTOR_DDR |= (1<<MOTOR_NEG) | (1<<MOTOR_PWM) | (1<<MOTOR_POS);
}

float getFloatValue(char* readBuffer, char* floatValuePointer) {
  floatValuePointer[0] = (char)readBuffer[3];
  floatValuePointer[1] = (char)readBuffer[2];
  floatValuePointer[2] = (char)readBuffer[1];
  floatValuePointer[3] = (char)readBuffer[0];
 
  return *(float*)floatValuePointer;
}

#endif