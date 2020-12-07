#ifndef MAP_SENSOR_H
#define MAP_SENSOR_H
#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>

class mapsensor
{
public:
    mapsensor( ModbusTCPServer *modbusTCPServer,
                int maddrMeasure,
                int maddrLowLimit1,
                int maddrHighLimit1,
                int maddrzeroSensor1,
                int maddrspanSensor1,
                int mConstNormalizacion);

    void mapFloatMeasurementSensor(int rawValueInputModule);
    void mapFloatMeasurementSensorInt(int rawValueInputModule);

    double getValueSensor();
    double getValueSensorNormaliced();

    ~mapsensor();
private:

    ModbusTCPServer* _modbusTCPServer;
    ModbusTCPClient* _modbusTCPClient1;
    ModbusTCPClient* _modbusTCPClient2;

    AnalogFilter<100, 10>* filterSensor;

    double calculatedSensorValues;
    double valueNormalization;

    int addrLowLimit1;
    int addrHighLimit1;
    int addrzeroSensor1;
    int addrspanSensor1;
    int constNormalizacion;
    int addrMeasure;
};


# endif