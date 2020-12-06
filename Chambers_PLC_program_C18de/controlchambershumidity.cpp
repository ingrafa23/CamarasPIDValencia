#include "controlchambershumidity.h"
#include "constPID.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"
#include "mapsensor.h"

controlchambershumidity::controlchambershumidity(ModbusTCPServer *modbusTCPServer,int maddressOffset){
    _modbusTCPServer = modbusTCPServer;
    addressOffset = maddressOffset;
    //-----------------
    _mapsensor = new mapsensor(&modbusTCPServer,
                        addressOffset + 266,            // CO2 Measure
                        addressOffset + 79,             // LowLimit1
                        addressOffset + 80,             // HighLimit1
                        addressOffset + 81,             // zeroSensor1
                        addressOffset + 92,             // spanSensor1
                        CONST_NORMALIZATION_HUMIDITY_PID);   // constante de normalización co2
    //-----------------
    //--------------------------------

  humidityPID = new PID(&valueHumidityNormalization, //----> Setpoint normalizado entre 0 y 1
                        &humidityPIDOutput,
                        &valueHumiditySetpointNormalization, //----> setpoint normalizado valor entre 0 y 1
                        1, 1, 1,
                        DIRECT);
  humidityPID->SetSampleTime(SAMPLE_TIME_HUMIDITY);
  humidityPID->SetOutputLimits(OUTPUT_HUMIDITY_LIMITS_MIN, OUTPUT_HUMIDITY_LIMITS_MAX);

  //------------------------
  humidityDownActivator = false;
  pidCycleControlHumidity = 0;
  alarmSensorHumidity1 = false;
  alarmSensorHumidity2 = false;
  flagForcedHumidity = 0;

}

void controlchambershumidity::alarm(){

}

//Control PID Humidity         
void controlchambershumidity::humidityControl(bool autoSelectorValue){

    /*BEGIN CONDITION ENABALE CONTROL SYSTEM HUMIDITY*/
    //Si la cámara está activa regula la humedad y si no desactiva las electroválvulas
    if (autoSelectorValue &&
        !_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) &&
        _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) &&
        alarmSensorHumidity1 == false && alarmSensorHumidity2 == false)
    {
      float relativeError = (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 14) -
                            _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 266)) /
                            _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 14);
      

      if (relativeError >=
          (1.0 - (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 137) / 100.0)))
      {
        
        humidityPID->SetMode(MANUAL);
        digitalWrite(HUMIDITY_WATER_VALVES, HIGH);
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 0);
        
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 336, 0);
        pidCycleControlHumidity = 1;
        humidityCycleTOn = 600;
        humidityCycleTOff = 10;
      }
      else if (relativeError <= 0)
      {
        
        humidityPID->SetMode(MANUAL);
        digitalWrite(HUMIDITY_WATER_VALVES, LOW);
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0);
        
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 336, 0);
        pidCycleControlHumidity = 0;
      }
      else
      {
        humidityPID->SetMode(AUTOMATIC);
        pidCycleControlHumidity = 1;
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 0);
      

      humidityPID->SetTunings((double)_modbusTCPServer->holdingRegisterRead(addressOffset + 43) / CONST_DIVISION_KP_HUMIDITY,
                              (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 44) / CONST_DIVISION_KI_HUMIDITY,
                              (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 45) / CONST_DIVISION_KD_HUMIDITY);



      humidityPID->SetOutputLimits(OUTPUT_HUMIDITY_LIMITS_MIN, _modbusTCPServer->holdingRegisterRead(addressOffset + 139));
      valueHumiditySetpointNormalization = _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 14) / CONST_NORMALIZATION_HUMIDITY_PID;
      humidityPID->Compute();

      
      humidityCycleTOn = (int)humidityPIDOutput;
      humidityCycleTOff = _modbusTCPServer->holdingRegisterRead(addressOffset + 139) -
                          (int)humidityPIDOutput;


      
      if (pidCycleControlHumidity == 1 && humidityPIDOutput >= 2)
      {

          if (*(humidityInyectionTimesPointer + *humidityInyectionStatusPointer) <= 0)
          {
            if (*humidityInyectionStatusPointer <= 0)
            {
              *(humidityInyectionTimesPointer + 1) = humidityCycleTOn;
              *humidityInyectionTimesPointer = humidityCycleTOff;
            }

            *humidityInyectionStatusPointer ^= 1;

            digitalWrite(HUMIDITY_WATER_VALVES, *humidityInyectionStatusPointer);
            
          }
        
      }
      else
      {      
        humidityPID->SetMode(MANUAL);
        digitalWrite(HUMIDITY_WATER_VALVES, LOW);
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0);
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 336, 0);
        pidCycleControlHumidity = 0;
      }
      }
    }
    else
    {
      humidityPID->SetMode(MANUAL);
      digitalWrite(HUMIDITY_WATER_VALVES, LOW);
      pidCycleControlHumidity = 0;
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0);
    }
    
  /*END CONDITION ENABALE CONTROL SYSTEM HUMIDITY*/

}


void controlchambershumidity::run(double medidaSendor){

}

void controlchambershumidity::enable(){

}

void controlchambershumidity::forced(){

}


void controlchambershumidity::writeIO(){

}

bool controlchambershumidity::getAlarmOnGeneral(){

}

void controlchambershumidity::readHumidity(double medidaSensor){
    _mapsensor->mapFloatMeasurementSensor(medidaSensor);
    valueHumidityNormalization = _mapsensor->getValueSensorNormaliced();
    calculatedSensorValues = _mapsensor->getValueSensor();
}

int controlchambershumidity::getAnalogOutputModule1ValuesHumidity(unsigned char _pos){

}

void controlchambershumidity::stateIndicator(void){

}

controlchambershumidity::~controlchambershumidity()
{
}