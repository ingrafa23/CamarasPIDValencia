/**
  @file VariableTimer.h
  @brief contiene todas las structura de las variables de los timer y control de los tiempos
  @author 
  @date 10/2020
*/
#ifndef VARIABLES_TIMER_INCLUDE
#define VARIABLES_TIMER_INCLUDE
#include <stdio.h>
#include <string.h>
#include <Arduino.h>

struct structTimer
{
    unsigned long timerInit;
    unsigned long timerOnDoor01;
    unsigned long timerOnDoor02;
    unsigned long timerOnFuga;
    unsigned long timerInyeccionFuga;
} ;


struct structMicroCut
{
    unsigned long timer10;
    unsigned long timer03;
    unsigned char flag10;
    unsigned char flag03;
} ;

//--timer Alarmas CO2-----
extern int timerGoOffAlarmCO2;
extern int timerLimitAlarmCO2;
//------------------------

//----timer alarma humedad----
extern int timerGoOffAlarmhumidity;
extern int timerLimitAlarmHumidity;

extern bool humidityInyectionStatus;
extern bool *humidityInyectionStatusPointer;

extern int humidityInyectionTimes[2];
extern int* humidityInyectionTimesPointer;
//----------------------------

//---Timer de Ethylene
extern int timerGoOffAlarmEthylene;
extern int timerLimitAlarmEthylene;
//-----------------------

//timer temperatura
extern int timerLimitAlarmTemperature;
extern int timerGoOffAlarmTemperature;
///-------------------


extern struct structMicroCut timerMicroCut;
extern struct structTimer timerInyectionEthyleneFlow;

extern void setTimerIntEthyleneFlow(unsigned long v);
extern void setTimerOnDoor01EthyleneFlow(unsigned long v);
extern void setTimerOnDoor02EthyleneFlow(unsigned long v);
extern void setTimerOnFugaEthyleneFlow(unsigned long v);
extern void setTimerInyeccionFugaEthyleneFlow(unsigned long v);

extern unsigned long getTimerIntEthyleneFlow();
extern unsigned long getTimerOnDoor01EthyleneFlow();
extern unsigned long getTimerOnDoor02EthyleneFlow();
extern unsigned long getTimerOnFugaEthyleneFlow();
extern unsigned long getTimerInyeccionFugaEthyleneFlow();


#endif
