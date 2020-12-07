#include "mapsensor.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"

mapsensor::mapsensor  (ModbusTCPServer *modbusTCPServer /*obj modbusTCPServer*/,
                        int maddrMeasure,
                        int maddrLowLimit1,
                        int maddrHighLimit1,
                        int maddrzeroSensor1,
                        int maddrspanSensor1,
                        int mConstNormalizacion)
{
  
  _modbusTCPServer = modbusTCPServer;

  addrMeasure = maddrMeasure;

  addrLowLimit1 = maddrLowLimit1;
  addrHighLimit1 = maddrHighLimit1;
  addrzeroSensor1 = maddrzeroSensor1;
  addrspanSensor1 = maddrspanSensor1;
  constNormalizacion = mConstNormalizacion;

  filterSensor = new AnalogFilter<100, 10>;

}

void mapsensor::mapFloatMeasurementSensor(int rawValueInputModule /*lectura del sensor*/){

    int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addrLowLimit1);
    int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addrHighLimit1);
    int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addrzeroSensor1);
    int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addrspanSensor1);
    int filteredMeasure = filterSensor->update(rawValueInputModule);
    calculatedSensorValues = mapFloat(filteredMeasure, zeroSensor1, spanSensor1, LowLimit1, HighLimit1);
    _modbusTCPServer->holdingRegisterWriteFloat(addrMeasure, calculatedSensorValues);
    valueNormalization = calculatedSensorValues / constNormalizacion;
}

void mapsensor::mapFloatMeasurementSensorInt(int rawValueInputModule /*lectura del sensor*/){

    int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addrLowLimit1);
    int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addrHighLimit1);
    int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addrzeroSensor1);
    int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addrspanSensor1);
    int filteredMeasure = filterSensor->update(rawValueInputModule);
    calculatedSensorValues = mapFloat(filteredMeasure, zeroSensor1, spanSensor1, LowLimit1, HighLimit1);
    _modbusTCPServer->holdingRegisterWriteFloat(addrMeasure, (int)calculatedSensorValues);
    valueNormalization = calculatedSensorValues / constNormalizacion;
}

double mapsensor::getValueSensor(){
    return calculatedSensorValues;
}

double mapsensor::getValueSensorNormaliced(){
    return valueNormalization;
}

mapsensor::~mapsensor(){}
