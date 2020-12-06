#ifndef CONTROL_CHAMBERS_HUMIDITY_H
#define CONTROL_CHAMBERS_HUMIDITY_H

#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>
#include "consoladebug.h"
#include "mapsensor.h"


class controlchambershumidity
{
private:

    double humiditySetpoint, humidityPIDOutput;
    PID* humidityPID;
    bool pidCycleControlHumidity;

    bool humidityDownActivator;
    float humidityPreviusValue;
    bool alarmSensorHumidity1;
    bool alarmSensorHumidity2;

    int humidityCycleTOn;
    int humidityCycleTOff;

    bool previusMode; //esta se pasa por referencia
    bool turnOn;

    bool flagForcedHumidity;

    double calculatedSensorValues;

    bool alarmOnGeneral;
    //Modbus
    ModbusTCPServer* _modbusTCPServer;
    //---
    mapsensor* _mapsensor;
    //---
    int addressOffset;

    //------
    //estrucura de los pines IO
    
    struct mcontrolchambershumidityIO
    {
        bool humidityWaterValves;
    }controlchambershumidityIO;

    void debugControlHumidity(String mdebug);

public:
    controlchambershumidity(ModbusTCPServer *modbusTCPServer,int maddressOffset);
    void setup();
    void run(double medidaSendor,bool autoSelectorValue);
    void enable();
    void forced();
    void alarm();
    void writeIO();
    bool getAlarmOnGeneral();
    //Control PID CO2         
    void humidityControl();
    void readHumidity(double medidaSensor);
    int getAnalogOutputModule1ValuesHumidity(unsigned char _pos);
    void stateIndicator(void);

    void setHumidityDownActivator(bool state);
    bool getHumidityDownActivator();

    ~controlchambershumidity();
};




#endif