#include "controlchambershumidity.h"
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
                        addressOffset + 266,            // humidity Measure
                        addressOffset + 79,             // LowLimit1
                        addressOffset + 80,             // HighLimit1
                        addressOffset + 81,             // zeroSensor1
                        addressOffset + 82,             // spanSensor1
                        CONST_NORMALIZATION_HUMIDITY_PID);   // constante de normalización humidity
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

  if (previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) && turnOn == false)
  {
    humidityDownActivator = false;
  }

  turnOn = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0);
  //Actualización del valor de modo anterior
  previusMode = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1);
  
  //////////activadores y desactivadores de las alarmas/////////
  if (calculatedSensorValues > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 36))
  {
    humidityDownActivator = true;
  }
  //////////acaban activadores y desactivadores de las alarmas/////////
 
  //Alarma repetición sensor de humedad

  //EMPIEZAN ALARMAS HUMEDAD
  if (calculatedSensorValues > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 34))
  {
    if (timerLimitAlarmHumidity <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 253, 15);
      alarmOnGeneral = true;
    }
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 253, 15);
    timerLimitAlarmHumidity = _modbusTCPServer->holdingRegisterRead(addressOffset + 157);
  }

  if (humidityDownActivator == true)
  {
    if (calculatedSensorValues[1] < _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 36))
    {
      if (timerLimitAlarmHumidity <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 0);
        alarmOnGeneral = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 0);
      timerLimitAlarmHumidity = _modbusTCPServer->holdingRegisterRead(addressOffset + 157);
    }
  }
  //ACABAN ALARMAS HUMEDAD HUMEDAD SOLO INFERIOR


  if (calculatedSensorValues == humidityPreviusValue) //--------------calculatedSensorValues[1]
  {
    if (timerGoOffAlarmhumidity <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 14);
      alarmSensorHumidity1 = true;
      alarmOnGeneral = true;
    }
  }
  else
  {
    timerGoOffAlarmhumidity = _modbusTCPServer->holdingRegisterRead(addressOffset + 153);

    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 14);
    alarmSensorHumidity1 = false;
  }

  humidityPreviusValue = calculatedSensorValues; ///-----------------calculatedSensorValues[1]
  //--------------------------------------
  //Alarma fallo sensor humedad
  if (calculatedSensorValues < _modbusTCPServer->holdingRegisterRead(addressOffset + 147)) ///-----------------calculatedSensorValues[1]
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 6);
    alarmSensorHumidity2 = true;
    alarmOnGeneral = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 6);
    alarmSensorHumidity2 = false;
  }
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

        controlchambershumidityIO.humidityWaterValves = 1; //digitalWrite(HUMIDITY_WATER_VALVES, HIGH);

        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 0);
        
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 336, 0);
        pidCycleControlHumidity = 1;
        humidityCycleTOn = 600;
        humidityCycleTOff = 10;
      }
      else if (relativeError <= 0)
      {
        
        humidityPID->SetMode(MANUAL);

        controlchambershumidityIO.humidityWaterValves = 0; // digitalWrite(HUMIDITY_WATER_VALVES, LOW);

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
      humiditySetpoint =_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 14);
      valueHumiditySetpointNormalization = humiditySetpoint / CONST_NORMALIZATION_HUMIDITY_PID;
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

            controlchambershumidityIO.humidityWaterValves = *humidityInyectionStatusPointer; //digitalWrite(HUMIDITY_WATER_VALVES, *humidityInyectionStatusPointer);
            
          }
        
      }
      else
      {      
        humidityPID->SetMode(MANUAL);

        controlchambershumidityIO.humidityWaterValves = 0; //digitalWrite(HUMIDITY_WATER_VALVES, LOW);

        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0);
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 336, 0);
        pidCycleControlHumidity = 0;
      }
      }
    }
    else
    {
      humidityPID->SetMode(MANUAL);

      controlchambershumidityIO.humidityWaterValves = 0; // digitalWrite(HUMIDITY_WATER_VALVES, LOW);

      pidCycleControlHumidity = 0;
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0);
    }
    
  /*END CONDITION ENABALE CONTROL SYSTEM HUMIDITY*/

}


void controlchambershumidity::run(double medidaSendor,bool autoSelectorValue){
  this->readHumidity(medidaSendor);
  this->alarm(); //aqui puede haber un error pero no se
  this->humidityControl(autoSelectorValue);
  this->enable();
  this->forced();
  this->writeIO();
  this->stateIndicator();

  //------Funcion que ejecuta si esta activo el debuger----------
  String strDebug;
  strDebug = F("-------Console humidity -------------------------------------\n");
  strDebug +=F("Setpoint : "); 
  strDebug += String(humiditySetpoint,DEC);
  strDebug = F("\n");
  strDebug +=F("Sensor Input  : "); 
  strDebug += String(calculatedSensorValues,DEC);
  strDebug = F("\n");
  strDebug +=F("humidityCycleTOn: "); 
  strDebug += String(humidityCycleTOn,DEC);
  strDebug = F("\n");
  strDebug +=F("humidityCycleTOff: "); 
  strDebug += String(humidityCycleTOff,DEC);
  strDebug = F("\n");
  strDebug = F("--------------------------------------------\n");
  this->debugControlHumidity(strDebug);
  //----------------------------------------------------------------


}

void controlchambershumidity::enable(){
  if(!ENABLE_HUMIDITY_WATER_VALVES){
    controlchambershumidityIO.humidityWaterValves = 0;
  }

}

void controlchambershumidity::forced(){
  if(FORCED_HUMIDITY_WATER_VALVES){
    controlchambershumidityIO.humidityWaterValves = 1;
  }
}


void controlchambershumidity::writeIO(){
  digitalWrite(HUMIDITY_WATER_VALVES, controlchambershumidityIO.humidityWaterValves);
}

bool controlchambershumidity::getAlarmOnGeneral(){
  return alarmOnGeneral;
}

void controlchambershumidity::readHumidity(double medidaSensor){
    _mapsensor->mapFloatMeasurementSensor(medidaSensor);
    valueHumidityNormalization = _mapsensor->getValueSensorNormaliced();
    calculatedSensorValues = _mapsensor->getValueSensor();
}



void controlchambershumidity::stateIndicator(void){
  //Salida HUMIDITY_WATER_VALVES
    if (digitalRead(HUMIDITY_WATER_VALVES))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 0)
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0)
    }
}


/* funcion para debugear el control de humedad */
void controlchambershumidity::debugControlHumidity(String mdebug){
  if (debugConsole.humidity)
    {
      unsigned long timeCosoleIn = millis() - debugLastTime.humidity;
      if (timeCosoleIn > 1000) //para que se imprima cada 1000ms
      {
        debugLastTime.humidity = millis();
        //--------------aca se imprime todo lo que quiera
        Serial.println(mdebug);
      }
    }
}


void controlchambershumidity::setHumidityDownActivator(bool state){
  humidityDownActivator = state;
}
bool controlchambershumidity::getHumidityDownActivator(){
  return humidityDownActivator;
}


controlchambershumidity::~controlchambershumidity()
{
}