#ifndef control_chamber_ethylene_H
#define control_chamber_ethylene_H

#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>
#include "consoladebug.h"
#include "mapsensor.h"


class controlChamberEthylene
{
private:
    //Modbus
    ModbusTCPServer* _modbusTCPServer;
    int addressOffset;
    //---
    mapsensor* _mapsensor;
    //---
    PID* ethylenePID;
    
    double ethyleneSetpoint, ethylenePIDOutput;

    bool ethylDownActivator;
    bool ethylUpActivator;

    float ethylPreviusValue;

    bool previusMode; //esta se pasa por referencia

    bool turnOn;
    bool alarmSensorEthyl1;
    bool alarmSensorEthyl2;
    //banderas de forzado del sistema de control 
    bool flagForcedEthylene;
    //banderas de enable del sistema de control 
    bool flagEnableControlSystemEthylene;
    //funciones privadas para hacer debuggear en etileno
    void debugControlEthylene(String mdebug);

    //Variables de normalizacion
    double valueEthyleneNormalization;  
    double valueEthyleneSetpointNormalization;
    double calculatedSensorValues;
    bool alarmOnGeneral;


public:
    controlChamberEthylene(ModbusTCPServer *modbusTCPServer,int maddressOffset);

    void setup();
    
    void run(double medidaSendor);
    void enable();
    void forced();
    void alarm();
    void writeIO();
    bool getAlarmOnGeneral();
    
    //Control PID de ethyleneControl
    void ethyleneControl(int* ethyleneInyectionTimesPointer,
                         bool* ethyleneInyectionStatusPointer);
    
    void readEthylene(double medidaSensor);
    int getAnalogOutputModule1ValuesEthylene(unsigned char _pos);
    void stateIndicator(void);

    bool getPreviusMode();
    void setEthylUpActivator(bool state);

    ~controlChamberEthylene();
}