#include "functions.h"

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void configureTimer5()
{
  //Configure Timer 5 to control timer events and set overflow interruption
  TCCR5A = 0x00;
  TCCR5B = 0x05;
  TCNT5H = 0xC2;
  TCNT5L = 0xF7;
  cli();
  TIMSK5 = 0x01;
  sei();
}

void setBitEeprom(int eepromAddress, int bit)
{
  byte data;
  data = EEPROM.read(eepromAddress);
  data |= (1 << bit);
  EEPROM.write(eepromAddress, data);
}

void clearBitEeprom(int eepromAddress, int bit)
{
  byte data;
  data = EEPROM.read(eepromAddress);
  data &= ~(1 << bit);
  EEPROM.write(eepromAddress, data);
}
