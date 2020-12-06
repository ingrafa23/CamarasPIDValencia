#include "readsensor.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"

readsensor::readsensor(ModbusTCPClient *modbusTCPClient1)
{
    _modbusTCPClient1 = modbusTCPClient1;
}


void readsensor::runReadSesor(){

  _modbusTCPClient1->requestFrom(HOLDING_REGISTERS, 40, 8);

  int counter = 0;
  while (_modbusTCPClient1->available())
  {
    rawValueInputModule1[counter] = _modbusTCPClient1->read();
    counter++;
  }
}

double readsensor::getValueSensor(int numSensor){
    return rawValueInputModule1[numSensor];
}