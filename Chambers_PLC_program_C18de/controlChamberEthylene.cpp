#include "controlchamberethylene.h"
#include "constPID.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"
#include "mapsensor.h"


void controlchamberethylene::setup(){

}

void controlchamberethylene::controlchamberethylene(ModbusTCPServer *modbusTCPServer,int maddressOffset)
{
  _modbusTCPServer = modbusTCPServer;
  addressOffset = maddressOffset;

//-----------------
  _mapsensor = new mapsensor(&modbusTCPServer,
                        addressOffset + 262,            // CO2 Measure
                        addressOffset + 83,             // LowLimit1
                        addressOffset + 84,             // HighLimit1
                        addressOffset + 85,             // zeroSensor1
                        addressOffset + 96,             // spanSensor1
                        CONST_NORMALIZATION_ETHYLENE_PID);   // constante de normalización co2
  //-----------------
  

  ethylenePID = new PID(&valueEthyleneNormalization,
                        &ethylenePIDOutput,
                        &valueEthyleneSetpointNormalization,
                        1, 1, 1,
                        DIRECT);
  ethylenePID->SetSampleTime(1000);
  
  ethylenePID->SetOutputLimits(ETHYLENE_PID_LIMIT_MIN, ETHYLENE_PID_LIMIT_MAX);
  ValidationPIDControlethylene.cont = 0;
  ValidationPIDControlethylene.valor = 0;
  ValidationPIDControlethylene.flag = 1;

  previusMode = false;

  ethylDownActivator = false;
  ethylUpActivator = false;
  turnOn = false;
  alarmSensorEthyl1 = false;
  alarmSensorEthyl2 = false;
  //banderas de forzado del sistema de control 
  flagForcedEthylene = 0;
  //banderas de enable del sistema de control 
  flagEnableControlSystemEthylene = 0;
  
}

void controlchamberethylene::alarm(){
    if (previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) && turnOn == false)
    {
        ethylUpActivator = false;
    }
    
    turnOn = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0);

    if ((previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1)) && previusMode == false)
    {
        ethylDownActivator = false;
    }
    if ((previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1)) && previusMode == true)
    {
        ethylUpActivator = false;
    }    

    if (calculatedSensorValues > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 40))
    {
        ethylDownActivator = true;
    }

    //////////acaban activadores y desactivadores de las alarmas/////////
    alarmOnGeneral = false;

    //EMPIEZAN ALARMAS ETILENO
    if (calculatedSensorValues > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 38))
    {
        if (timerLimitAlarmEthylene <= 0)
        {
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 1);
            alarmOnGeneral = true;
        }
    }
    else
    {
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 1);
        timerLimitAlarmEthylene = _modbusTCPServer->holdingRegisterRead(addressOffset + 158);
    }

    if (ethylDownActivator == true)
    {
        if (calculatedSensorValues < _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 40))
        {
            if (timerLimitAlarmEthylene <= 0)
            {
                _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 2);
                alarmOnGeneral = true;
            }
        }
        else
        {
            _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 2);
            timerLimitAlarmEthylene = _modbusTCPServer->holdingRegisterRead(addressOffset + 158);
        }
    }

    //Actualización del valor de modo anterior
    previusMode = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1);

    /////////////////////////Alarmas de bloqueo de sensor////////////////////////////
    //Alarma repetición sensor de etileno
  
    if (calculatedSensorValues == ethylPreviusValue)
    {
        if (timerGoOffAlarmEthylene <= 0)
        {
            //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 4);
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 255, 0);
            alarmSensorEthyl1 = true;
            alarmOnGeneral = true;
        }
    }
    else
    {
        timerGoOffAlarmEthylene = _modbusTCPServer->holdingRegisterRead(addressOffset + 154);
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 255, 0);
        alarmSensorEthyl1 = false;
    }

    ethylPreviusValue = calculatedSensorValues;

    ///////////////////////ALARMAS FALLO SENSORES//////////////////////
    //Alarma fallo sensor etileno
    if (calculatedSensorValues < _modbusTCPServer->holdingRegisterRead(addressOffset + 149) ||
        calculatedSensorValues > _modbusTCPServer->holdingRegisterRead(addressOffset + 148))
    {
        //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 4);
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 8);
        alarmSensorEthyl2 = true;
        alarmOnGeneral = true;
    }
    else
    {
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 8);
        alarmSensorEthyl2 = false;
    }

}


void controlchamberethylene::readEthylene(double medidaSendor){
  _mapsensor->mapFloatMeasurementSensor(medidaSensor);
  valueEthyleneNormalization = _mapsensor->getValueSensorNormaliced();
  calculatedSensorValues = _mapsensor->getValueSensor();
}


void controlchamberethylene::ethyleneControl(){

}

void controlchamberethylene::run(){
}

