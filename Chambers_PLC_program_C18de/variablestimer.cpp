#include "variablestimer.h"

structTimer timerInyectionEthyleneFlow;
struct structTimer timerMicroCut = {0,0,0,0};

//timer Alarmas CO2
int timerGoOffAlarmCO2 = 0;
int timerLimitAlarmCO2 = 0;

void setTimerIntEthyleneFlow(unsigned long v){
    timerInyectionEthyleneFlow.timerInit = v;
}
void setTimerOnDoor01EthyleneFlow(unsigned long v){
    timerInyectionEthyleneFlow.timerOnDoor01 = v;
}
void setTimerOnDoor02EthyleneFlow(unsigned long v){
    timerInyectionEthyleneFlow.timerOnDoor02 = v;
}
void setTimerOnFugaEthyleneFlow(unsigned long v)
{
    timerInyectionEthyleneFlow.timerOnFuga = v;
}
void setTimerInyeccionFugaEthyleneFlow(unsigned long v){
    timerInyectionEthyleneFlow.timerInyeccionFuga = v;
}

unsigned long getTimerIntEthyleneFlow(){
    return timerInyectionEthyleneFlow.timerInit;
}
unsigned long getTimerOnDoor01EthyleneFlow(){
    return timerInyectionEthyleneFlow.timerOnDoor01;
}
unsigned long getTimerOnDoor02EthyleneFlow(){
    return timerInyectionEthyleneFlow.timerOnDoor02;
}
unsigned long getTimerOnFugaEthyleneFlow(){
    return timerInyectionEthyleneFlow.timerOnFuga;
}
unsigned long getTimerInyeccionFugaEthyleneFlow(){
    return timerInyectionEthyleneFlow.timerInyeccionFuga;
};
