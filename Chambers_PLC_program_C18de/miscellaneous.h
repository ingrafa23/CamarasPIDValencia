#ifndef MISCELLANEOUS_H
#define MISCELLANEOUS_H

#include <avr/interrupt.h>
#include <Arduino.h>
#include <EEPROM.h>

struct StructValidationPID
{
  long valor;
  int cont;
  bool flag;
};

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void configureTimer5();

void setBitEeprom(int eepromAddress, int bit);

void clearBitEeprom(int eepromAddress, int bit);

//****************************************************************************************


//Fornulas del sistema
double formulaInyecionInicial(double mREF_C2H4, double mCHAMBER_VOLUMEN);

// Filtro de validacion de las salidas del PID
#define numeValidation 10 // numero de muestras para la validacion
void ValidationPID(StructValidationPID *v, long limtC);

#endif 
