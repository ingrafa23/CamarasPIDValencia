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



extern structTimer timerInyectionEthyleneFlow;

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