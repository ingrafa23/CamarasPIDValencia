#include "variablestimer.h"

struct structMicroCut timerMicroCut;
struct structTimer timerInyectionEthyleneFlow;


//timer Alarmas CO2
int timerGoOffAlarmCO2 = 0;
int timerLimitAlarmCO2 = 0;

//----timer alarma humedad---- esprobable que se coloque esto en el ino
int timerGoOffAlarmhumidity=0;
int timerLimitAlarmHumidity = 0;

bool humidityInyectionStatus = 0;
bool *humidityInyectionStatusPointer = &humidityInyectionStatus;

int humidityInyectionTimes[2] = {0, 0};
int* humidityInyectionTimesPointer = &humidityInyectionTimes[0];
//----------------------------

//---Timer de Ethylene
int timerGoOffAlarmEthylene = 0;
int timerLimitAlarmEthylene = 0;
//-----------------------

//timer temperatura
int timerLimitAlarmTemperature = 0;
int timerGoOffAlarmTemperature = 0;
///------------------------------

//timer alarmas generales
int timerAlarmNoVentilation = 0;
int timerOpenDoorTimeAlarm1 = 0;
int timerOpenDoorTimeAlarm2 = 0;

//-----------Timer activar extractores por temperatura
struct strTimerExtrActTemp timerActivarExtractoresTemperatura = {0,0};

// variables alarm open door 01
bool flagTimerOpenDoorTimeAlarm1Pointer;
// variables alarm open door 01
bool flagTimerOpenDoorTimeAlarm2Pointer;


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
