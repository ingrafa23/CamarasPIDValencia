#ifndef READ_SENSOR_H
#define READ_SENSOR_H
#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>

#define NUMBER_SENSOR 8

class readsensor
{
public:
    readsensor(ModbusTCPClient *modbusTCPClient1);

    void runReadSesor(void);
    double getValueSensor(int numSensor);

    ~readsensor();
private:

    int rawValueInputModule1[NUMBER_SENSOR];
    ModbusTCPClient* _modbusTCPClient1;
    
};


#endif
