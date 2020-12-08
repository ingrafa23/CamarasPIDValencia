#ifndef CONTROL_CHAMBER_TEMPERATURE_
#define CONTROL_CHAMBER_TEMPERATURE_H
#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>
#include "mapsensor.h"

class controlChamberTemperature
{
private:
    /* data */
        //Modbus
    ModbusTCPServer* _modbusTCPServer;
    int addressOffset;
    //---
    mapsensor* _mapsensor;
    //---
    bool pidCycleControlTemp = 0;

    bool previusMode = false;
    bool turnOn = false;

    bool tempUpActivator = false;
    bool tempDownActivator = false;

    float tempPreviusValue;

    bool alarmSensorTemp1 = false;

    bool alarmSensorTemp2 = false;

    double calculatedSensorValues;

    bool alarmOnGeneral;

    struct mcontrolChamberTemperatureIO
    {
        bool ControlCoolingRequest;
        bool ControlHeatingRequest;
        bool HeatingRequest;
        bool CoolingRequest;
        bool Aeroheaters;

    }controlChamberTemperatureIO;

    void debugControlTemperature();

   
    double temperaturaExterior;
    bool activadoresVentiladores;
     bool flagReturnActivadoresVentiladores;
    
    


public:
    controlChamberTemperature(ModbusTCPServer *modbusTCPServer,int maddressOffset);
    void setup();
    void run(double medidaSensor,bool autoSelectorValue,double valorTemperaturaExterior,bool flagActivadoresVentiladores);
    void barrido();
    bool getForceVentiladores();
    void enable();
    void forced();
    void alarm();
    void writeIO();
    bool getAlarmOnGeneral();
    //Control        
    void TemperatureControl(bool autoSelectorValue);
    void readTemperature(double medidaSensor);
    void stateIndicator(void);

    ~controlChamberTemperature();
};



#endif