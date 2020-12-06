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
    ModbusTCPServer* _modbusTCPServer;
    ModbusTCPClient* _modbusTCPClient1;
    ModbusTCPClient* _modbusTCPClient2;
    PID* ethylenePID;
    AnalogFilter<100, 10>* filterEthyl;

    int _chamber;
    int addressOffset;
    int eepromOffset;
    int numHoldingRegistersAddresses = 370;
    int numEepromAddresses = 500;
    int rawValueInputModule1[8];
    double calculatedSensorValues[8];
    int analogOutputModule1Values[4];

    double ethyleneSetpoint, ethylenePIDOutput;

    bool ethylDownActivator = false;
    bool ethylUpActivator = false;

    float ethylPreviusValue;

    bool alarmSensorEthyl1 = false;

    bool alarmSensorEthyl2 = false;

    //banderas de forzado del sistema de control 
    bool flagForcedEthylene = 0;

    //banderas de enable del sistema de control 
    bool flagEnableControlSystemEthylene = 0;

    //Activador alarma
    bool alarmOn = 0;

    //funciones privadas para hacer debuggear en etileno
    void debugControlEthylene();

public:
    controlChamberEthylene(int chamber,
                            ModbusTCPServer* modbusTCPServer,
                            ModbusTCPClient* modbusTCPClient1,
                            ModbusTCPClient* modbusTCPClient2,
                            int &holdingRegisterPerChamber);

    void setup();
    void run(double medidaSendor);
    void enable();
    void forced();
    void alarm();
    void writeIO();
    bool getAlarmOnGeneral();
    
    //Control PID de ethyleneControl
    ethyleneControl(int* ethyleneInyectionTimesPointer,
                    bool* ethyleneInyectionStatusPointer);
    
    readEthylene(double medidaSensor);
    int getAnalogOutputModule1ValuesEthylene(unsigned char _pos);
    void stateIndicator(void);

    ~controlChamberEthylene();
}