#ifndef CONTROL_CHAMBER_ETHYLENE_H
#define CONTROL_CHAMBER_ETHYLENE_H

#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>
#include "consoladebug.h"
#include "mapsensor.h"

//ETHYLENE PID CONTROL
  #define CONST_NORMALIZATION_ETHYLENE_PID 1
  //#define ETHYLENE_PID_LIMIT_MIN 13500 //minimo PID
  #define ETHYLENE_PID_LIMIT_MIN 13500 //minimo PID
  #define ETHYLENE_PID_LIMIT_MAX 15000//16200 //Maximo PID
  #define ETHYLENE_PID_CLOSE 4000 // minimo forzado
  #define ETHYLENE_PID_OPEN 20000 // maximo forzado 


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
    double valueEthyleneNormalization; 
    double valueEthyleneSetpointNormalization; 
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
    
    
    double calculatedSensorValues;
    bool alarmOnGeneral;

    struct StructValidationPID ValidationPIDControlethylene;


    //Salidas
    struct controlchamberethyleneAnalogOutputModule1
    {
        int value;
        bool flag;
    }analogOutputModule1Values;
    
       
    //funciones privadas para hacer debuggear en etileno
    void debugControlEthylene(String mdebug);



public:
    controlChamberEthylene(ModbusTCPServer *modbusTCPServer,int maddressOffset);

    void setup();
    void run(double medidaSensor);
    void enable();
    void forced();
    void alarm();
    bool getAlarmOnGeneral();

    double getValueEthylene();
    
    //Control PID de ethyleneControl
    void ethyleneControl();
    
    void readEthylene(double medidaSensor);
    int getAnalogOutputModule1ValuesEthylene();
    bool getAnalogOutputModule1FlagEthylene();
    void stateIndicator(void);

    ~controlChamberEthylene();
};

#endif