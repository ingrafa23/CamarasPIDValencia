#include <avr/interrupt.h>
#include <Arduino.h>
#include <EEPROM.h>

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void configureTimer5();

void setBitEeprom(int eepromAddress, int bit);

void clearBitEeprom(int eepromAddress, int bit);
