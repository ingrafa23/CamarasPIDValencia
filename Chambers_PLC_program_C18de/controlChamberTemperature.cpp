#include "controlChamberTemperature.h"

#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"
#include "mapsensor.h"

controlChamberTemperature::controlChamberTemperature(ModbusTCPServer *modbusTCPServer,int maddressOffset){
    _modbusTCPServer = modbusTCPServer;
    addressOffset = maddressOffset;

    //-----------------
    _mapsensor = new mapsensor(_modbusTCPServer,
                        addressOffset + 351,             // Measure
                        addressOffset + 75,             // LowLimit1
                        addressOffset + 76,             // HighLimit1
                        addressOffset + 77,             // zeroSensor1
                        addressOffset + 78,             // spanSensor1
                        1);   // constante de normalización co2
    //-----------------

}


void controlChamberTemperature::setup(){

}  

/**
 * @brief run es la funcion principal que ejecuta todo el sistema de control de temperatura
 * @param medidaSensor es el valor de la medida del sensor
 * @param autoSelectorValue habilita el control de temperatura

*/

void controlChamberTemperature::run(double medidaSensor,bool autoSelectorValue){
    this->readTemperature(medidaSensor);
    this->alarm();
    this->TemperatureControl(autoSelectorValue);
    this->enable();
    this->forced();
    this->writeIO();
    this->stateIndicator();

    this->debugControlTemperature();

    
}

void controlChamberTemperature::enable(){

  if(!ENABLE_COOLING_REQUEST){
      controlChamberTemperatureIO.CoolingRequest = 0;
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 12);
    
  }

  if(!ENABLE_HEATING_REQUEST){
    controlChamberTemperatureIO.HeatingRequest = 0;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 13);
  }

  if(!ENABLE_CONTROL_COOLING_REQUEST){
    controlChamberTemperatureIO.ControlCoolingRequest = 0;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 8);
    
  }

  if(!ENABLE_CONTROL_HEATING_REQUEST){
    controlChamberTemperatureIO.ControlHeatingRequest = 0;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
  }

  if(!ENABLE_AEROHEATERS){

    controlChamberTemperatureIO.Aeroheaters = 0;
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, 0x00);
  }

}

void controlChamberTemperature::forced(){
  if(FORCED_COOLING_REQUEST){
      controlChamberTemperatureIO.CoolingRequest = 1;
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 12);
    
  }

  if(FORCED_HEATING_REQUEST){
    controlChamberTemperatureIO.HeatingRequest = 1;
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 13);
  }

  if(FORCED_CONTROL_COOLING_REQUEST){
    controlChamberTemperatureIO.ControlCoolingRequest = 1;
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 8);
    
  }

  if(FORCED_CONTROL_HEATING_REQUEST){
    controlChamberTemperatureIO.ControlHeatingRequest = 1;
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 9);
  }

  if(FORCED_AEROHEATERS){
    controlChamberTemperatureIO.Aeroheaters = 1;
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, 0xff);
  }
}

void controlChamberTemperature::alarm()
{
  if (previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) && turnOn == false)
  {
    tempDownActivator = false;
    tempUpActivator = false;
  }
  
  turnOn = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0);

  if ((previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1)) && previusMode == false)
  { 
    tempDownActivator = false;
  }
    
  if ((previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1)) && previusMode == true)
  { 
    tempUpActivator = false;
  }

  if (calculatedSensorValues < _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 30))
  {
    tempUpActivator = true;
  }

  if (calculatedSensorValues > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 32))
  {
    tempDownActivator = true;
  }

  /////////acaban activadores y desactivadores de las alarmas/////////
  alarmOnGeneral = false;
    
  ////////////////EMPIEZAN ALARMAS LÍMITES/////////////////////////
  //EMPIEZAN ALARMAS TEMPERATURA
  if (digitalRead(DEFROST_CYCLE) == false && tempUpActivator == true)
  {
    if (calculatedSensorValues>= _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 30))
    {
      if (timerLimitAlarmTemperature <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 253, 13);
        alarmOnGeneral = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 253, 13);
      timerLimitAlarmTemperature = _modbusTCPServer->holdingRegisterRead(addressOffset + 156);
    }
  }

  if (tempDownActivator == true)
  {
    if (calculatedSensorValues <= _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 32))
    {
      if (timerLimitAlarmTemperature <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 253, 14);
        alarmOnGeneral = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 253, 14);
      timerLimitAlarmTemperature = _modbusTCPServer->holdingRegisterRead(addressOffset + 156);
    }
  }
  //ACABAN ALARMAS TEMPERATURA

  ////////////////////////Alarmas de bloqueo de sensor////////////////////////////
  //Alarma repetición sensor de temperatura
  if (calculatedSensorValues == tempPreviusValue)
  {
    if (timerGoOffAlarmTemperature <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 12);
      alarmSensorTemp1 = true;
      alarmOnGeneral = true;
    }
  }
  else
  {
    timerGoOffAlarmTemperature = _modbusTCPServer->holdingRegisterRead(152);

    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 12);
    alarmSensorTemp1 = false;
  }
  tempPreviusValue = calculatedSensorValues;
  
  ///////////////////////ALARMAS FALLO SENSORES//////////////////////
  //Alarma fallo sensor temperatura
  if (calculatedSensorValues < _modbusTCPServer->holdingRegisterRead(addressOffset + 146) ||
      calculatedSensorValues > _modbusTCPServer->holdingRegisterRead(addressOffset + 145))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 4);
    alarmSensorTemp2 = true;
    alarmOnGeneral = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 4);
    alarmSensorTemp2 = false;
  }
}

void controlChamberTemperature::writeIO(){
    digitalWrite(CONTROL_COOLING_REQUEST,controlChamberTemperatureIO.ControlCoolingRequest);
    digitalWrite(CONTROL_HEATING_REQUEST,controlChamberTemperatureIO.ControlHeatingRequest);
    digitalWrite(HEATING_REQUEST,controlChamberTemperatureIO.HeatingRequest);
    digitalWrite(COOLING_REQUEST,controlChamberTemperatureIO.CoolingRequest);
    digitalWrite(AEROHEATERS,controlChamberTemperatureIO.Aeroheaters);
}

bool controlChamberTemperature::getAlarmOnGeneral(){
    return alarmOnGeneral;
}
//Control        

void controlChamberTemperature::TemperatureControl(bool autoSelectorValue){
    
  ///// Temperature external control selected /////
  if (autoSelectorValue &&
      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) &&
      alarmSensorTemp1 == false && alarmSensorTemp2 == false)
  {

    ///////// Prepare the external equipment to work //////////
    //Prepare to warm up
    if (calculatedSensorValues <
        (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 26)))
    {

      controlChamberTemperatureIO.ControlCoolingRequest = 0; //----digitalWrite(CONTROL_COOLING_REQUEST, LOW);
      controlChamberTemperatureIO.ControlHeatingRequest = 1;//-----------digitalWrite(CONTROL_HEATING_REQUEST, HIGH);

      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 8);
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 9);
    }
    //Prepapare to cool down
    if (calculatedSensorValues >
        (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 28)))
    {
         
      controlChamberTemperatureIO.ControlHeatingRequest = 0; //---------digitalWrite(CONTROL_HEATING_REQUEST, LOW);
     
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
      /////////////////
      
    }
    //////////////////////////////////////////////////

    ///////////// Temperature control/////////////////
    /////Heating activation//////
    if ((calculatedSensorValues <=
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 22)) &&
        digitalRead(EXT_HEATER_AVAILABLE))
    {
        
     controlChamberTemperatureIO.HeatingRequest =1; //----------digitalWrite(HEATING_REQUEST, HIGH);

      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 13);
    }

    if (!digitalRead(EXT_HEATER_AVAILABLE))
    {
        
        controlChamberTemperatureIO.HeatingRequest = 0; //----------digitalWrite(HEATING_REQUEST, LOW);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 13);
    }

    ////Cooling activation////
    if ((calculatedSensorValues >=
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 24)) &&
        digitalRead(EXT_COOLER_AVAILABLE))
    {
        

      controlChamberTemperatureIO.CoolingRequest = 1; //---------------digitalWrite(COOLING_REQUEST, HIGH);
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 12);
      Serial.println("Activado El Ciclo Enfriado");
    }

    if (!digitalRead(EXT_COOLER_AVAILABLE))
    {
        
      controlChamberTemperatureIO.CoolingRequest = 0; //---------------digitalWrite(COOLING_REQUEST, LOW);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 12);
    }

    ///// Stop the actions when temeperature is inside range////
    if ((calculatedSensorValues >
         (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
          _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 26))) &&
        (calculatedSensorValues <
         (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
          _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 28))))
    {
      controlChamberTemperatureIO.HeatingRequest = 0; //----------digitalWrite(HEATING_REQUEST, LOW);
      controlChamberTemperatureIO.CoolingRequest = 0; //---------------digitalWrite(COOLING_REQUEST, LOW);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 12);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 13);
    }
    ////////////////////////////////////////////////////

    //////// Control the input signal between systems////////

    if (digitalRead(EXT_HEATER_AVAILABLE))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 11);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 11);
    }

    if (digitalRead(EXT_COOLER_AVAILABLE))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 10);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 10);
    }

    if (digitalRead(HEATER_ACTIVATED))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 15);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 15);
    }

    if (digitalRead(COOLER_ACTIVATED))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 14);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 14);
    }

    if (digitalRead(DEFROST_CYCLE))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 251, 0);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 251, 0);
    }
    ///////////////////////////////////////////////////
  }
  else
  {

    ////Swicht off all acctions and requests////
    controlChamberTemperatureIO.HeatingRequest = 0; //----------digitalWrite(HEATING_REQUEST, LOW);
    controlChamberTemperatureIO.CoolingRequest = 0; //---------------digitalWrite(COOLING_REQUEST, LOW);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 13);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 12);

    controlChamberTemperatureIO.ControlHeatingRequest = 0; //---------digitalWrite(CONTROL_HEATING_REQUEST, LOW);
    controlChamberTemperatureIO.ControlCoolingRequest = 0; //----digitalWrite(CONTROL_COOLING_REQUEST, LOW);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 8);
  }

  ////// Heating with aeroheaters //////
  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) &&
      alarmSensorTemp1 == false && alarmSensorTemp2 == false)
  {
    if (calculatedSensorValues <=
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 22))
    {
      
      controlChamberTemperatureIO.Aeroheaters = 1; //----------digitalWrite(AEROHEATERS, HIGH);
      uint16_t value = _modbusTCPServer->holdingRegisterRead(addressOffset + 259);
      value |= 0xFF;
      _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, value);
      
    }

    if (calculatedSensorValues >=
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 26))
    {
        
      controlChamberTemperatureIO.Aeroheaters = 0; //----------digitalWrite(AEROHEATERS, LOW);
      uint16_t value = _modbusTCPServer->holdingRegisterRead(addressOffset + 259);
      value &= ~0xFF;
      _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, value);
    }
  }
  else
  {
    controlChamberTemperatureIO.Aeroheaters = 0; //----------digitalWrite(AEROHEATERS, LOW);
    uint16_t value = _modbusTCPServer->holdingRegisterRead(addressOffset + 259);
    value &= ~0xFF;
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, value);

  }

  //////////////////////////////////////
}

void controlChamberTemperature::readTemperature(double medidaSensor){
    _mapsensor->mapFloatMeasurementSensor(medidaSensor);
    _mapsensor->mapFloatLimitador(addressOffset + 270);
   calculatedSensorValues = _mapsensor->getValueSensor();

}

void controlChamberTemperature::stateIndicator(void){
  if (controlChamberTemperatureIO.CoolingRequest)
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 339, 0);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 339, 0);
  }
//----------------------
  if (controlChamberTemperatureIO.HeatingRequest)
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 15);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 15);
  }
//-------------------------
  if (controlChamberTemperatureIO.ControlCoolingRequest)
  {
    //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 9);
  }
  else
  {
    //_modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
  }
//-----------------------
  if (controlChamberTemperatureIO.ControlHeatingRequest)
  {
    //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 9);
  }
  else
  {
    //_modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
  }
//----------------------------
  if (controlChamberTemperatureIO.Aeroheaters)
  {
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, 0xff);
  }
  else
  {
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, 0x00);
  }

//------------------------------------------------------
  if (digitalRead(DEFROST_CYCLE))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 339, 2);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 339, 2);
  }
  

}

//funciones privadas

/* funcion para debugear el control de temperatura */
void controlChamberTemperature::debugControlTemperature(){
  if (debugConsole.temp)
  {
    unsigned long timeConsoleIn = abs(millis() - debugLastTime.temp);
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
      debugLastTime.temp = millis();
      //--------------aca se imprime todo lo que quiera
      Serial.println("-------Console temperature -----------------------------");
      Serial.print("Sensor temperature value : "); Serial.println(calculatedSensorValues);
      Serial.print("Threshold heating activator : "); Serial.println(_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
                                                                     _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 22) &&
                                                                     digitalRead(EXT_HEATER_AVAILABLE));
      Serial.print("Heating activation : "); Serial.println(HEATING_REQUEST);
      Serial.print("Threshold cooling activator : "); Serial.println(_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
                                                                     _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 24) &&
                                                                     digitalRead(EXT_COOLER_AVAILABLE));
      Serial.print("Cooling activation : "); Serial.println(COOLING_REQUEST);       
      Serial.println("-------________________---------------------------------");
    }
  }
}


controlChamberTemperature::~controlChamberTemperature()
{
}