#include "controlChambers.h"
#include "constPID.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"

unsigned long lastValueInputFan1;
unsigned long lastValueOutputFan1;
unsigned long lastValueInputFan2;
unsigned long lastValueOutputFan2;
bool flagTimerAlarmNoVentilationPointer;


bool flagTimerOpenDoorTimeAlarm1Pointer;
bool flagTimerOpenDoorTimeAlarm2Pointer;

unsigned int lastTimeGases = 0;
unsigned int TimeGases;


struct strHoldingRegisterControlEnable holdingRegisterControlEnable;

struct StructValidationPID  ValidationPIDFlowethylene, ValidationPIDControlethylene, ValidationPIDCO2;



Chamber::Chamber(int chamber,
                 ModbusTCPServer *modbusTCPServer,
                 ModbusTCPClient *modbusTCPClient1,
                 ModbusTCPClient *modbusTCPClient2,
                 int &holdingRegisterPerChamber)
{
  _chamber = chamber;
  _modbusTCPServer = modbusTCPServer;
  _modbusTCPClient1 = modbusTCPClient1;
  _modbusTCPClient2 = modbusTCPClient2;

  holdingRegisterPerChamber += numHoldingRegistersAddresses;
  addressOffset = _chamber * numHoldingRegistersAddresses;
  eepromOffset = _chamber * numEepromAddresses;

  humidityPID = new PID(&valueHumidityNormalization, //----> Setpoint normalizado entre 0 y 1
                        &humidityPIDOutput,
                        &valueHumiditySetpointNormalization, //----> setpoint normalizado valor entre 0 y 1
                        1, 1, 1,
                        DIRECT);
  humidityPID->SetSampleTime(SAMPLE_TIME_HUMIDITY);
  humidityPID->SetOutputLimits(OUTPUT_HUMIDITY_LIMITS_MIN, OUTPUT_HUMIDITY_LIMITS_MAX);

  ethylenePID = new PID(&valueEthyleneNormalization,
                        &ethylenePIDOutput,
                        &valueEthyleneSetpointNormalization,
                        1, 1, 1,
                        DIRECT);
  ethylenePID->SetSampleTime(1000);


  
  ethylenePID->SetOutputLimits(ETHYLENE_PID_LIMIT_MIN, ETHYLENE_PID_LIMIT_MAX);


  // PID Control CO2
  CO2PID = new PID(&valueCO2Normalization,
                   &CO2PIDOutput,
                   &valueCO2SetpointNormalization,
                   1, 1, 1,
                   REVERSE);
  CO2PID->SetSampleTime(50);
  CO2PID->SetOutputLimits(PID_CO2_CONTROL_MIN, PID_CO2_CONTROL_MAX);

  ValidationPIDCO2.cont = 0;
  ValidationPIDCO2.valor = 0;
  ValidationPIDCO2.flag = 1;

  // end PID Control CO2

  // PID Control ethyleneFlowPID
  ethyleneFlowPID = new PID(&valueEthyleneFlowNormalization,
                            &ethyleneFlowPIDOutput,
                            &valueEthyleneFlowSetpointNormalization,
                            1, 1, 1,
                            DIRECT);
                            
  

  ethyleneFlowPID->SetSampleTime(100);
  ethyleneFlowPID->SetOutputLimits(ETHYLENE__FLOW_PID_LIMIT_MIN, ETHYLENE__FLOW_PID_LIMIT_MAX);

  // end PID Control ethyleneFlowPID

  ValidationPIDFlowethylene.cont = 0;
  ValidationPIDFlowethylene.valor = 0;
  ValidationPIDFlowethylene.flag = 1;

  ValidationPIDControlethylene.cont = 0;
  ValidationPIDControlethylene.valor = 0;
  ValidationPIDControlethylene.flag = 1;

  filterEthylFlow = new AnalogFilter<100, 10>;
  filterTemp = new AnalogFilter<100, 10>;
  filterHum = new AnalogFilter<100, 10>;
  filterCO2 = new AnalogFilter<100, 10>;
  filterEthyl = new AnalogFilter<100, 10>;

  

}

Chamber::init(int *timerAlarmNoVentilationPointer,
              int *timerOpenDoorTimeAlarm1Pointer,
              int *timerOpenDoorTimeAlarm2Pointer)
{

  //inicializo los timer de alamr a de puertas y ventilador 
  *timerAlarmNoVentilationPointer = hour2seg(MAX_TIME_ALARM_NO_VENTILATION);
  *timerOpenDoorTimeAlarm1Pointer = MAX_TIME_OPEN_DOOR_1;
  *timerOpenDoorTimeAlarm2Pointer = MAX_TIME_OPEN_DOOR_2;

  //Get persisten variables from eeprom
  int counter = 0;

  tempDownActivator = false;
  ethylDownActivator = false;
  humidityDownActivator = false;
  tempUpActivator = false;

  for (int i = 0; i < 250; i++)
  {
    uint16_t value = 0;
    uint8_t highByte = EEPROM.read(eepromOffset + counter + 1);
    uint8_t lowByte = EEPROM.read(eepromOffset + counter);
    value = highByte << 8;
    value += lowByte;
    _modbusTCPServer->holdingRegisterWrite(addressOffset + i, value);
    counter += 2;
  }

  if (EEPROM.read(0) & (1 << 3))
  {
    digitalWrite(Q0_0, HIGH);
    delay(100);
    digitalWrite(Q0_0, LOW);
    clearBitEeprom(0, 3);
  }


  
}

Chamber::writeToEeprom()
{
  int modbusAddress = _modbusTCPServer->holdingRegisterRead(addressOffset + 261);
  int dataType = _modbusTCPServer->holdingRegisterRead(addressOffset + 260);
  int eepromAddress = _modbusTCPServer->holdingRegisterRead(addressOffset + 261) * 2;

  if (dataType == 1)
  {
    EEPROM.put(eepromOffset + eepromAddress,
               _modbusTCPServer->holdingRegisterRead(addressOffset + modbusAddress));
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 261, 0);
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 260, 0);
    Serial.print("value stored: ");
    Serial.println(_modbusTCPServer->holdingRegisterRead(addressOffset + modbusAddress));
    Serial.println("Register stored in eeprom");
  }

  if (dataType == 2)
  {
    float floatValue = _modbusTCPServer->holdingRegisterReadFloat(addressOffset + modbusAddress);
    uint16_t words[2];

    modbus_set_float_dcba(floatValue, words);

    for (int i = 0; i < 2; i++)
    {
      EEPROM.put((eepromOffset + eepromAddress + 2 * i), words[i]);
    }

    _modbusTCPServer->holdingRegisterWrite(addressOffset + 261, 0);
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 260, 0);

    Serial.println("Float stored in eeprom");
  }

  //  if (dataType == 3)
  //  {
  //    long longValue = _modbusTCPServer->holdingRegisterReadLong(addressOffset + modbusAddress);
  //    uint8_t bytes[4];
  //
  //    bytes[0] = (uint8_t) longValue;
  //    bytes[1] = (uint8_t) longValue >> 8;
  //    bytes[2] = (uint8_t) longValue >> 16;
  //    bytes[3] = (uint8_t) longValue >> 24;
  //
  //    for (int i = 0; i < 4; i++)
  //    {
  //      EEPROM.put((eepromOffset + eepromAddress + i), bytes[i]);
  //    }
  //
  //    _modbusTCPServer->holdingRegisterWrite(addressOffset + 261, 0);
  //    _modbusTCPServer->holdingRegisterWrite(addressOffset + 260, 0);
  //
  //    Serial.println("Long stored in eeprom");
  //  }
}

Chamber::temperatureControl()
{

  

          
  ///// Temperature external control selected /////
  if (autoSelectorValue &&
      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) &&
      alarmSensorTemp1 == false && alarmSensorTemp2 == false && 
      flagEnableControlSystemTemperatureExternal)
  {

    ///////// Prepare the external equipment to work //////////
    //Prepare to warm up
    if (calculatedSensorValues[0] <
        (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 26)))
    {

      digitalWrite(CONTROL_COOLING_REQUEST, LOW);
      digitalWrite(CONTROL_HEATING_REQUEST, HIGH);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 8);
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 9);
    }
    //Prepapare to cool down
    if (calculatedSensorValues[0] >
        (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 28)))
    {
      digitalWrite(CONTROL_HEATING_REQUEST, LOW);
      digitalWrite(CONTROL_COOLING_REQUEST, HIGH);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 8);
    }
    //////////////////////////////////////////////////

    ///////////// Temperature control/////////////////
    /////Heating activation//////
    if ((calculatedSensorValues[0] <=
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 22)) &&
        digitalRead(EXT_HEATER_AVAILABLE))
    {
      digitalWrite(HEATING_REQUEST, HIGH);

      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 13);
    }

    if (!digitalRead(EXT_HEATER_AVAILABLE))
    {
      digitalWrite(HEATING_REQUEST, LOW);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 13);
    }

    ////Cooling activation////
    if ((calculatedSensorValues[0] >=
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
         _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 24)) &&
        digitalRead(EXT_COOLER_AVAILABLE))
    {
      digitalWrite(COOLING_REQUEST, HIGH);
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 12);
      Serial.println("Activado El Ciclo Enfriado");
    }

    if (!digitalRead(EXT_COOLER_AVAILABLE))
    {
      digitalWrite(COOLING_REQUEST, LOW);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 12);
    }

    ///// Stop the actions when temeperature is inside range////
    if ((calculatedSensorValues[0] >
         (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
          _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 26))) &&
        (calculatedSensorValues[0] <
         (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
          _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 28))))
    {
      digitalWrite(HEATING_REQUEST, LOW);
      digitalWrite(COOLING_REQUEST, LOW);
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
    digitalWrite(HEATING_REQUEST, LOW);
    digitalWrite(COOLING_REQUEST, LOW);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 13);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 12);

    digitalWrite(CONTROL_HEATING_REQUEST, LOW);
    digitalWrite(CONTROL_COOLING_REQUEST, LOW);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 9);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 8);
  }

  ////// Heating with aeroheaters //////
  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) &&
      alarmSensorTemp1 == false && alarmSensorTemp2 == false &&
      flagEnableControlSystemTemperatureAerotermo)
  {
    if (calculatedSensorValues[0] <=
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 22))
    {
      digitalWrite(AEROHEATERS, HIGH);

      uint16_t value = _modbusTCPServer->holdingRegisterRead(addressOffset + 259);
      value |= 0xFF;
      _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, value);
    }

    if (calculatedSensorValues[0] >=
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
        _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 26))
    {

      digitalWrite(AEROHEATERS, LOW);
      uint16_t value = _modbusTCPServer->holdingRegisterRead(addressOffset + 259);
      value &= ~0xFF;
      _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, value);
    }
  }
  else
  {
    digitalWrite(AEROHEATERS, LOW);
    uint16_t value = _modbusTCPServer->holdingRegisterRead(addressOffset + 259);
    value &= ~0xFF;
    _modbusTCPServer->holdingRegisterWrite(addressOffset + 259, value);

  }

  //////////////////////////////////////
}

Chamber::humidityControl(int *humidityInyectionTimesPointer, bool *humidityInyectionStatusPointer)
{
  /*BEGIN CONDITION ENABALE CONTROL SYSTEM HUMIDITY*/
    //Si la cámara está activa regula la humedad y si no desactiva las electroválvulas
    if (autoSelectorValue &&
        !_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) &&
        _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) &&
        alarmSensorHumidity1 == false && alarmSensorHumidity2 == false &&
        flagEnableControlSystemHumidity)
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
      else if (relativeError <= (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 137) / 100) - 1.0)
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
      }

      humidityPID->SetTunings((double)_modbusTCPServer->holdingRegisterRead(addressOffset + 43) / CONST_DIVISION_KP_HUMIDITY,
                              (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 44) / CONST_DIVISION_KI_HUMIDITY,
                              (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 45) / CONST_DIVISION_KD_HUMIDITY);



      humidityPID->SetOutputLimits(OUTPUT_HUMIDITY_LIMITS_MIN, _modbusTCPServer->holdingRegisterRead(addressOffset + 139));
      valueHumiditySetpointNormalization = _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 14) / CONST_NORMALIZATION_HUMIDITY_PID;
      humidityPID->Compute();

      
      humidityCycleTOn = (int)humidityPIDOutput;
      humidityCycleTOff = _modbusTCPServer->holdingRegisterRead(addressOffset + 139) -
                          (int)humidityPIDOutput;

      //------Funcion que ejecuta si esta activo el debuger----------
      debugControlHumidity();
      //----------------
      
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
    else
    {
      humidityPID->SetMode(MANUAL);
      digitalWrite(HUMIDITY_WATER_VALVES, LOW);
      pidCycleControlHumidity = 0;
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 0);
    }
    
  /*END CONDITION ENABALE CONTROL SYSTEM HUMIDITY*/
}

Chamber::ethyleneControl(int *ethyleneInyectionTimesPointer, bool *ethyleneInyectionStatusPointer)
{
   
  /*BEGIN CONDITION ENABALE CONTROL SYSTEM ETHYLENE*/

    ////////////////////Modo desverdizado activo///////////////////////
    //Modo trabajo desverdización
    if ( _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) ) 
            //Cámara en marcha
    {

      /////////////////Bloque control por PID//////////////////////////
      if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) &&
          _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4))
      {
        if(alarmSensorEthyl1 == false && alarmSensorEthyl2 == false  &&
          flagEnableControlSystemEthylene) //Control por análisis activo
        {
          //Serial.println("Analysis");

          float relativeError = (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) -
                                _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 262)) /
                                _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18);

          if (relativeError >= (1 - (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 140) / 100)))
          {
            
            ethylenePID->SetMode(MANUAL);
            analogOutputModule1Values[2] = ETHYLENE_PID_OPEN;
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
            
          }
          else if (relativeError <= (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 140) / 100) - 1)
          {
            
            ethylenePID->SetMode(MANUAL);
            analogOutputModule1Values[2] = ETHYLENE_PID_CLOSE;
            _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
            
            
          }
          else
          {
            
            ethylenePID->SetMode(AUTOMATIC);
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
            
            ethyleneSetpoint = (double)_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18);
            valueEthyleneSetpointNormalization = ethyleneSetpoint / CONST_NORMALIZATION_ETHYLENE_PID;

            

            ethylenePID->SetTunings(KP_ETHYLENE_PID, KI_ETHYLENE_PID, KD_ETHYLENE_PID);

            
            ethylenePID->Compute();
            analogOutputModule1Values[2] = ethylenePIDOutput;
            

            ValidationPIDControlethylene.valor = ethylenePIDOutput;

            ValidationPID(&ValidationPIDControlethylene, ETHYLENE_PID_LIMIT_MIN);

            
            if (ValidationPIDControlethylene.flag)
            {
              analogOutputModule1Values[2] = ETHYLENE_PID_CLOSE;
              _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
              
            }

            //------Funcion que ejecuta si esta activo el debuger----------
            debugControlEthylene();
            //----------------

            



          }
          
        }
        else
        {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 4);
        }
      }

      ///////////////////////Fin bloque de control por análisis//////////////////////////

    }
    else
    {
      ethylenePID->SetMode(MANUAL);
      analogOutputModule1Values[2] = 4000;
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
      
    }

  /*END CONDITION ENABALE CONTROL SYSTEM ETHYLENE*/
}

Chamber::CO2Control(int *timerInitializationFanPointer)
{

  //Si la cámara está en marcha regula el CO2 y si no desactiva la regulación
  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) ||
      alarmSensorCO21 == false || alarmSensorCO22 == false || 
      (!flagEnableControlSystemCO2OnOff && !flagEnableControlSystemCO2Pid))  //Cámara en marcha
  {

    /////////////////////////Control de CO2 por consigna//////////////////////////
    if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 5) &&
        flagEnableControlSystemCO2OnOff) //Control de CO2 por consigna habilitado
    {
      if (calculatedSensorValues[3] >=
          _modbusTCPServer->holdingRegisterRead(addressOffset + 21))
      {
        analogOutputModule1Values[0] = 20000;
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 2);
        analogOutputModule1Values[1] = 20000;
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 3);

        
        //------------------------
        digitalWrite(INPUT_FAN_1, HIGH);
        indicadoresEstados.inputFan1 = 1;
        //------------------------
        
        //------------------------
        digitalWrite(OUTPUT_FAN_1, HIGH);
        indicadoresEstados.outputFan1 = 1;
        //------------------------
        
        //------------------------
        digitalWrite(INPUT_FAN_2, HIGH);
        indicadoresEstados.inputFan2 = 1;
        //------------------------
        
        //------------------------
        digitalWrite(OUTPUT_FAN_2, HIGH);
        indicadoresEstados.outputFan2 = 1;
        //------------------------
        
      }

      if (calculatedSensorValues[3] <=
          _modbusTCPServer->holdingRegisterRead(addressOffset + 20))
      {

        analogOutputModule1Values[0] = 4000;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
        analogOutputModule1Values[1] = 4000;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);

        //------------------------
        digitalWrite(INPUT_FAN_1, LOW);
        indicadoresEstados.inputFan1 = 0;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_1, LOW);
        indicadoresEstados.outputFan1 = 0;
        //------------------------
        //------------------------
        digitalWrite(INPUT_FAN_2, LOW);
        indicadoresEstados.inputFan2 = 0;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_2, LOW);
        indicadoresEstados.outputFan2 = 0;
        //------------------------
      }
    }
    /////////////////////Final Control de CO2 por consigna////////////////////////

    //////////////////////////Control del CO2 por PID/////////////////////////////
    if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 5) &&
        flagEnableControlSystemCO2Pid)
    {
      //Si la medida es menor que el límite inferior se desconecta el PID
      
      if (calculatedSensorValues[3] <= _modbusTCPServer->holdingRegisterRead(addressOffset + 20))
      {
        CO2PID->SetMode(MANUAL);
        analogOutputModule1Values[0] = 4000;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
        analogOutputModule1Values[1] = 4000;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
        
        //------------------------
        digitalWrite(INPUT_FAN_1, LOW);
        indicadoresEstados.inputFan1 = 0;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_1, LOW);
        indicadoresEstados.outputFan1 = 0;
        //------------------------
        //------------------------
        digitalWrite(INPUT_FAN_2, LOW);
        indicadoresEstados.inputFan2 = 0;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_2, LOW);
        indicadoresEstados.outputFan2 = 0;
        //------------------------
      }
      else
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 2);
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 3);
        //------------------------
        digitalWrite(INPUT_FAN_1, HIGH);
        indicadoresEstados.inputFan1 = 1;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_1, HIGH);
        indicadoresEstados.outputFan1 = 1;
        //------------------------
        //------------------------
        digitalWrite(INPUT_FAN_2, HIGH);
        indicadoresEstados.inputFan2 = 1;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_2, HIGH);
        indicadoresEstados.outputFan2 = 1;
        //------------------------

        CO2PID->SetMode(AUTOMATIC);

    
        CO2PID->SetTunings(KP_CO2_PID, KI_CO2_PID, KD_CO2_PID, 1);

        CO2Setpoint = (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 21);
        valueCO2SetpointNormalization = CO2Setpoint / CONST_NORMALIZATION_CO2_PID; // linea nueva

      
        CO2PID->Compute();

        analogOutputModule1Values[0] = CO2PIDOutput;
        analogOutputModule1Values[1] = CO2PIDOutput;

        ValidationPIDCO2.valor = CO2PIDOutput;
        ValidationPID(&ValidationPIDCO2, PID_CO2_CONTROL_MIN);
        
        

        if (ValidationPIDCO2.flag)
        {
          analogOutputModule1Values[0] = CO2_PID_CLOSE;
          analogOutputModule1Values[1] = CO2_PID_CLOSE;
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
          
        }
        
        //------Funcion que ejecuta si esta activo el debuger----------
        debugControlCo2();
        //------------------------------------------------------------

        
      }
    }
    else
    {
        CO2PID->SetMode(MANUAL);
        analogOutputModule1Values[0] = 4000;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
        analogOutputModule1Values[1] = 4000;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
        //------------------------
        digitalWrite(INPUT_FAN_1, LOW);
        indicadoresEstados.inputFan1 = 0;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_1, LOW);
        indicadoresEstados.outputFan1 = 0;
        //------------------------
        //------------------------
        digitalWrite(INPUT_FAN_2, LOW);
        indicadoresEstados.inputFan2 = 0;
        //------------------------
        //------------------------
        digitalWrite(OUTPUT_FAN_2, LOW);
        indicadoresEstados.outputFan2 = 0;
        //------------------------
    }
    ///////////////////////Final Control del CO2 por PID//////////////////////////  


  }
  else
  {
    CO2PID->SetMode(MANUAL);
    ethyleneFlowRateCO2Control = 0;
    analogOutputModule1Values[0] = 4000;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
    analogOutputModule1Values[1] = 4000;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
    
    //------------------------
    digitalWrite(INPUT_FAN_1, LOW);
    indicadoresEstados.inputFan1 = 0;
    //------------------------
    //------------------------
        digitalWrite(OUTPUT_FAN_1, LOW);
        indicadoresEstados.outputFan1 = 0;
        //------------------------
    //------------------------
        digitalWrite(INPUT_FAN_2, LOW);
        indicadoresEstados.inputFan2 = 0;
        //------------------------
    //------------------------
        digitalWrite(OUTPUT_FAN_2, LOW);
        indicadoresEstados.outputFan2 = 0;
        //------------------------
  }
/*END CONDITION ENABALE CONTROL SYSTEM CO2*/

}



Chamber::ethyleneFlowRateControl()
{
  
/**
 * Codigo Nuevo
 * 
*/

//Habilitado de etileno
  /*BEGIN CONDITION ENABALE CONTROL SYSTEM ETHYLENE*/
    // Entrada modo trabajo desverdizacion
  
  
  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0))
  {
    if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1))     
    {
      modoConservacion();//Entrada en modo Conservacion
    }
    else //Entrada en modo Desverdizacion
    {
      modoDesverdizacion();
    }
    if(!flagEnableControlSystemEthylene && !flagEnableControlSystemEthyleneFlow) 
    {
      ethylenePID->SetMode(MANUAL);
      analogOutputModule1Values[2] = 4000;
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
    }
  }
  else
  {
  ethylenePID->SetMode(MANUAL);
  analogOutputModule1Values[2] = 4000;
  _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
  }



/**
 * 
 * fin de codigo nuevo
*/

}

Chamber::readTemp()
{

  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 75);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 76);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 77);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 78);
  int filteredMeasure = filterTemp->update(rawValueInputModule1[2]);
  calculatedSensorValues[0] = mapFloat(filteredMeasure, zeroSensor1, spanSensor1, LowLimit1, HighLimit1);
  _modbusTCPServer->holdingRegisterWriteFloat((addressOffset + 270), calculatedSensorValues[0]);
  
}

Chamber::readHumidity()
{
  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 79);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 80);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 81);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 82);
  int filteredMeasure = filterHum->update(rawValueInputModule1[3]);
  calculatedSensorValues[1] = mapFloat(filteredMeasure, zeroSensor1, spanSensor1, LowLimit1, HighLimit1);

  valueHumidityNormalization = calculatedSensorValues[1] / CONST_NORMALIZATION_HUMIDITY_PID;

  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 266, calculatedSensorValues[1]);
  
}

Chamber::readEthylene()
{
  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 83);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 84);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 85);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 86);
  int filteredMeasure = filterEthyl->update(rawValueInputModule1[1]);
  calculatedSensorValues[2] = mapFloat(filteredMeasure, zeroSensor1, spanSensor1, LowLimit1, HighLimit1);

  //Normalizacion
  valueEthyleneNormalization = calculatedSensorValues[2] / CONST_NORMALIZATION_ETHYLENE_PID;
  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 262, calculatedSensorValues[2]);
  
}

Chamber::readCO2()
{

  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 87);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 88);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 89);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 90);
  int filteredMeasure = filterCO2->update(rawValueInputModule1[0]);
  calculatedSensorValues[3] = mapFloat(filteredMeasure, zeroSensor1, spanSensor1, LowLimit1, HighLimit1);
  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 264, calculatedSensorValues[3]);

  //Normalizacion
  valueCO2Normalization = calculatedSensorValues[3] / CONST_NORMALIZATION_CO2_PID;
  //

}

Chamber::readEthyleneFlowRate()
{
  int rawMeasure = analogRead(ETHYLENE_FLOW_IN);

  int filterMeasure = filterEthylFlow->update(rawMeasure);
  
  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 115);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 116);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 117);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 118);
  calculatedSensorValues[4] = mapFloat(filterMeasure,
                                       zeroSensor1,
                                       spanSensor1,
                                       LowLimit1,
                                       HighLimit1);
  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 274, calculatedSensorValues[4]);
  
  valueEthyleneFlowNormalization = calculatedSensorValues[4]  / CONST_NORMALIZATION_ETHYLENE_FLOW_PID;
}

Chamber::readOutputFan1()
{
  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 119);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 120);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 121);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 122);
  calculatedSensorValues[5] = mapFloat(analogRead(OUTPUT_FAN_PHASE_1),
                                       zeroSensor1,
                                       spanSensor1,
                                       LowLimit1,
                                       HighLimit1);

  
  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 285, calculatedSensorValues[5]);
}

Chamber::readOutputFan2()
{
  int LowLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 123);
  int HighLimit1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 124);
  int zeroSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 125);
  int spanSensor1 = _modbusTCPServer->holdingRegisterRead(addressOffset + 126);
  calculatedSensorValues[6] = mapFloat(analogRead(OUT_FAN_PHASE_2),
                                       zeroSensor1,
                                       spanSensor1,
                                       LowLimit1,
                                       HighLimit1);
  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 289, calculatedSensorValues[6]);
}

Chamber::getMeasurements()
{
  readTemp();
  readHumidity();
  readEthylene();
  readCO2();
  readEthyleneFlowRate();
  readOutputFan1();
  readOutputFan2();
}

Chamber::getRawValues1()
{
  _modbusTCPClient1->requestFrom(HOLDING_REGISTERS, 40, 8);

  int counter = 0;
  while (_modbusTCPClient1->available())
  {
    rawValueInputModule1[counter] = _modbusTCPClient1->read();
    counter++;
  }
}

Chamber::writeAnalogValues()
{
  
  _modbusTCPClient2->beginTransmission(HOLDING_REGISTERS, 40, 4);

  for (int i = 0; i < 4; i++)
  {
    _modbusTCPClient2->write(analogOutputModule1Values[i]);
  }

  _modbusTCPClient2->endTransmission();
}

Chamber::alarms(int *timerGoOffAlarmTemperaturePointer,
                int *timerGoOffAlarmHumidityPointer,
                int *timerGoOffAlarmEthylenePointer,
                int *timerGoOffAlarmCO2Pointer,
                int *timerLimitAlarmTemperaturePointer,
                int *timerLimitAlarmHumidityPointer,
                int *timerLimitAlarmEthylenePointer,
                int *timerLimitAlarmCO2Pointer,
                int *timerAlarmNoVentilationPointer,
                int *timerOpenDoorTimeAlarm1Pointer,
                int *timerOpenDoorTimeAlarm2Pointer)
{

  
  //  Serial.println(calculatedSensorValues[0]);
  //  Serial.println(_modbusTCPServer->holdingRegisterRead(addressOffset + 75));
  //  Serial.println(_modbusTCPServer->holdingRegisterRead(addressOffset + 76));
  //  Serial.println(_modbusTCPServer->holdingRegisterRead(addressOffset + 77));
  //  Serial.println(_modbusTCPServer->holdingRegisterRead(addressOffset + 78));

  //  Serial.println(calculatedSensorValues[1]);
  //  Serial.println(calculatedSensorValues[2]);
  //  Serial.println(calculatedSensorValues[3]);

  //empiezan activatores y desactivatores de las alarmas
/* if (digitalRead(AUTO_TEL_SELECTOR)) 
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 8);
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 256, 11);


    //Avisa mediante mensaje escrito en holding register que no se puede
    //activar On/Off System


  }
  else {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 8);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 256, 11);


    if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 256, 10);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 256, 10);
    }
  }
*/

  //--------------------------------------------------------------



  if (previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) && turnOn == false)
  {
    tempDownActivator = false;
    ethylUpActivator = false;
    humidityDownActivator = false;
    tempUpActivator = false;
  }

  turnOn = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0);

  if ((previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1)) && previusMode == false)
  {
    tempDownActivator = false;
    ethylDownActivator = false;
  }
  if ((previusMode != _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1)) && previusMode == true)
  {
    tempUpActivator = false;
    ethylUpActivator = false;
  }

  if (calculatedSensorValues[0] < _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 30))
  {
    tempUpActivator = true;
  }

  if (calculatedSensorValues[0] > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 32))
  {
    tempDownActivator = true;
  }

  if (calculatedSensorValues[1] > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 36))
  {
    humidityDownActivator = true;
  }

  if (calculatedSensorValues[2] > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 40))
  {
    ethylDownActivator = true;
  }

  //////////acaban activadores y desactivadores de las alarmas/////////

  alarmOn = false;




  ////////////////EMPIEZAN ALARMAS LÍMITES/////////////////////////

  //EMPIEZAN ALARMAS TEMPERATURA

  if (digitalRead(DEFROST_CYCLE) == false && tempUpActivator == true)
  {
    if (calculatedSensorValues[0] >= _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 30))
    {
      if (*timerLimitAlarmTemperaturePointer <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 253, 13);
        alarmOn = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 253, 13);
      *timerLimitAlarmTemperaturePointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 156);
    }
  }

  if (tempDownActivator == true)
  {
    if (calculatedSensorValues[0] <= _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 32))
    {
      if (*timerLimitAlarmTemperaturePointer <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 253, 14);
        alarmOn = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 253, 14);
      *timerLimitAlarmTemperaturePointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 156);
    }
  }
  //ACABAN ALARMAS TEMPERATURA

  //EMPIEZAN ALARMAS HUMEDAD
  if (calculatedSensorValues[1] > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 34))
  {
    if (*timerLimitAlarmHumidityPointer <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 253, 15);
      alarmOn = true;
    }
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 253, 15);
    *timerLimitAlarmHumidityPointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 157);
  }

  if (humidityDownActivator == true)
  {
    if (calculatedSensorValues[1] < _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 36))
    {
      if (*timerLimitAlarmHumidityPointer <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 0);
        alarmOn = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 0);
      *timerLimitAlarmHumidityPointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 157);
    }
  }
  //ACABAN ALARMAS HUMEDAD HUMEDAD SOLO INFERIOR

  //EMPIEZAN ALARMAS ETILENO
  if (calculatedSensorValues[2] > _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 38))
  {
    if (*timerLimitAlarmEthylenePointer <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 1);
      alarmOn = true;
    }
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 1);
    *timerLimitAlarmEthylenePointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 158);
  }

  if (ethylDownActivator == true)
  {
    if (calculatedSensorValues[2] < _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 40))
    {
      if (*timerLimitAlarmEthylenePointer <= 0)
      {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 2);
        alarmOn = true;
      }
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 2);
      *timerLimitAlarmEthylenePointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 158);
    }
  }

  //ACABAN ALARMAS ETILENO

  //EMPIEZAN ALARMAS CO2
  if (calculatedSensorValues[3] > _modbusTCPServer->holdingRegisterRead(addressOffset + 42))
  {
    if (*timerLimitAlarmCO2Pointer <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 3);
      alarmOn = true;
      
    }
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 3);
    *timerLimitAlarmCO2Pointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 159);
  }
  //ACABAN ALARMAS CO2

  ////////////////ACABAN ALARMAS LÍMITES/////////////////////////

  //Actualización del valor de modo anterior
  previusMode = _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1);

  //Lectura de los neutros y las fases de los ventiladores de entrada y salida y tiempo neutro

  //  inFanPhase = analogRead(IN_FAN_PHASE);
  //  inFanPhase = mapFloat(inFanPhase, 0, 10.23, 0, 0.1);
  //  outFanPhase = analogRead(OUT_FAN_PHASE);
  //  outFanPhase = mapFloat(outFanPhase, 0, 10.23, 0, 0.1);
  //  inFanPhase2 = analogRead(IN_FAN_PHASE_2);
  //  inFanPhase2 = mapFloat(outFanPhase2, 0, 10.23, 0, 0.1);
  //  outFanPhase2 = analogRead(OUT_FAN_PHASE_2);
  //  outFanPhase2 = mapFloat(outFanPhase2, 0, 10.23, 0, 0.1);

  //Fin lectura de los neutros y las fases de los ventiladores de entrada y salida y tiempo neutro

  //Escribimos en modbus
  //  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 205, inFanPhase);
  //  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 207, outFanPhase);
  //  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 209, inFanPhase2);
  //  _modbusTCPServer->holdingRegisterWriteFloat(addressOffset + 211, outFanPhase2);

  /////////////////////////Alarmas de bloqueo de sensor////////////////////////////

  //Alarma repetición sensor de temperatura

  if (calculatedSensorValues[0] == tempPreviusValue)
  {
    if (*timerGoOffAlarmTemperaturePointer <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 12);
      alarmSensorTemp1 = true;
      alarmOn = true;
    }
  }
  else
  {
    *timerGoOffAlarmTemperaturePointer = _modbusTCPServer->holdingRegisterRead(152);

    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 12);
    alarmSensorTemp1 = false;
  }

  tempPreviusValue = calculatedSensorValues[0];

  //Alarma repetición sensor de humedad

  if (calculatedSensorValues[1] == humidityPreviusValue)
  {
    if (*timerGoOffAlarmHumidityPointer <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 14);
      alarmSensorHumidity1 = true;
      alarmOn = true;
    }
  }
  else
  {
    *timerGoOffAlarmHumidityPointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 153);

    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 14);
    alarmSensorHumidity1 = false;
  }

  humidityPreviusValue = calculatedSensorValues[1];

  //Alarma repetición sensor de etileno
  //sin * comparamos valor de la memoria con se compara con valor almacenado
  //calculatedSensorValues[2] = 2;
  if (calculatedSensorValues[2] == ethylPreviusValue)
  {
    //Serial.print("Entra repetición  :"); Serial.println(*timerGoOffAlarmEthylenePointer);

    if (*timerGoOffAlarmEthylenePointer <= 0)
    {
      //      Serial.println("Entra alarma");
      //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 4);
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 255, 0);
      alarmSensorEthyl1 = true;
      alarmOn = true;
    }
  }
  else
  {
    *timerGoOffAlarmEthylenePointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 154);
    //    Serial.println("cambio");
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 255, 0);
    alarmSensorEthyl1 = false;
  }

  ethylPreviusValue = calculatedSensorValues[2];

  //Alarma repetición sensor de CO2

  //Serial.println(calculatedSensorValues[3]);

  if (calculatedSensorValues[3] == CO2PreviusValue)
  {
    int e = *timerGoOffAlarmCO2Pointer;
    //Serial.println(e);

    if (*timerGoOffAlarmCO2Pointer <= 0)
    {
      //      Serial.println("repetido");
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 255, 2);
      alarmSensorCO21 = true;
      alarmOn = true;
    }
  }
  else
  {
    *timerGoOffAlarmCO2Pointer = _modbusTCPServer->holdingRegisterRead(addressOffset + 155);
    //    Serial.println("diferente");
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 255, 2);
    alarmSensorCO21 = false;
  }

  CO2PreviusValue = calculatedSensorValues[3];

  ///////////////////////ALARMAS FALLO SENSORES//////////////////////

  //Alarma fallo sensor temperatura
  if (calculatedSensorValues[0] < _modbusTCPServer->holdingRegisterRead(addressOffset + 146) ||
      calculatedSensorValues[0] > _modbusTCPServer->holdingRegisterRead(addressOffset + 145))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 4);
    alarmSensorTemp2 = true;
    alarmOn = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 4);
    alarmSensorTemp2 = false;
  }
  //Alarma fallo sensor humedad
  if (calculatedSensorValues[1] < _modbusTCPServer->holdingRegisterRead(addressOffset + 147))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 6);
    alarmSensorHumidity2 = true;
    alarmOn = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 6);
    alarmSensorHumidity2 = false;
  }
  //Alarma fallo sensor etileno
  if (calculatedSensorValues[2] < _modbusTCPServer->holdingRegisterRead(addressOffset + 149) ||
      calculatedSensorValues[2] > _modbusTCPServer->holdingRegisterRead(addressOffset + 148))
  {
    //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 4);
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 8);
    alarmSensorEthyl2 = true;
    alarmOn = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 8);
    alarmSensorEthyl2 = false;
  }
  //Serial.println(calculatedSensorValues[3]);

  //Alarma fallo sensor CO2

  if (calculatedSensorValues[3] < _modbusTCPServer->holdingRegisterRead(addressOffset + 151) ||
      calculatedSensorValues[3] > _modbusTCPServer->holdingRegisterRead(addressOffset + 150))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 10);
    alarmSensorCO22 = true;
    alarmOn = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 10);
    alarmSensorCO22 = false;
  }

  if(alarmOn == true)
  {
    digitalWrite(ALARM_SET, HIGH);
  }
  else
  {
    digitalWrite(ALARM_SET, LOW);
  }


  //EMPIEZAN VERIFICAR ALARMAS NO VENTILAR 
    if (*timerAlarmNoVentilationPointer > 0) {
    if( (VALUE_ACTUAL_INPUT_FAN1) || (VALUE_ACTUAL_OUTPUT_FAN1) || (VALUE_ACTUAL_INPUT_FAN2) || (VALUE_ACTUAL_OUTPUT_FAN2) ) {

     //unsigned int maxTimeAlarmVentilation = EEPROM.read(142)<<8 + EEPROM.read(143);
     *timerAlarmNoVentilationPointer = hour2seg(MAX_TIME_ALARM_NO_VENTILATION);
     flagTimerAlarmNoVentilationPointer = false;
    }
    else
    {
     if(*timerAlarmNoVentilationPointer == 0 ){
       flagTimerAlarmNoVentilationPointer = true;
       _modbusTCPServer->holdingRegisterSetBit(256, 9);
     }
    }
    }
  //FINALIZAR VERIFICACION ALARMAS VENTILAR

  //EMPIEZAN A VERIFICAR ALARMAS POR TIEMPO DE PUERTA ABIERTA
  // PUERTA 1
  if (VALUE_DOOR_1)
  {
    if (*timerOpenDoorTimeAlarm1Pointer == 0){
      flagTimerOpenDoorTimeAlarm1Pointer = true;
      alarmOn = true;
      _modbusTCPServer->holdingRegisterSetBit(338, 5);
    }
    else{
      flagTimerOpenDoorTimeAlarm1Pointer = false;
    }
  }
  else
  {
    *timerOpenDoorTimeAlarm1Pointer = MAX_TIME_OPEN_DOOR_1;
  } 
  
  // PUERTA 2
  if (VALUE_DOOR_2)
  {
    if (*timerOpenDoorTimeAlarm2Pointer == 0){
      flagTimerOpenDoorTimeAlarm2Pointer = true;
      alarmOn = true;
      _modbusTCPServer->holdingRegisterSetBit(338, 6);
    }
    else{
      flagTimerOpenDoorTimeAlarm2Pointer = false;
    }
  }
  else
  {
    *timerOpenDoorTimeAlarm2Pointer = MAX_TIME_OPEN_DOOR_2;
  } 
  //FINALIZAN VERIFICACION ALARMA POR TIEMPO DE PUERTA ABIERTA

}





///------------------------------------------------------------------------
/**
 * @brief modoDesverdizacion 
*/
void Chamber::modoDesverdizacion()
{
  //
  //Serial.print("Balance de gases :"); Serial.println(!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4));
  //Serial.print("flagEnableControlSystemEthyleneFlow :"); Serial.println(flagEnableControlSystemEthyleneFlow);
  
  if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4) //si balance de gases activado
          && flagEnableControlSystemEthyleneFlow) //SI las alarmas saltan entra en balance de gases
      {
        //
        //Serial.print("Hay modificaciones del PID:"); Serial.println(_modbusTCPServer->holdingRegisterReadBit(addressOffset + 250, 4));
        //Hay modificaciones del PID        
        if(_modbusTCPServer->holdingRegisterReadBit(addressOffset + 250, 4))
        { //inyecion inicial inicializacion

          //Pone activador inyección inicial etileno a 0
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 4);
                   
          // Inicio de conservacion de ciclo activado
          //Paramos inyeccion por mantenimiento
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 2); 

          //Este es el timer de inyección le cargamos el valor de inyección inicial
          //*(ethyleneInyectionTimesPointer + 1) = _modbusTCPServer->holdingRegisterRead(addressOffset + 59);
          setTimerIntEthyleneFlow(_modbusTCPServer->holdingRegisterRead(addressOffset + 59));
          //Este es el timer de tiempo sin inyectar le cargamos el valor de 0
          //*ethyleneInyectionStatusPointer = 0;
          estadoModoDesverdizacion = MODO_INYECCION_INICIAL;
          desiredEthyleneFlowRate = 0;
          double baseEthyleneFlowRate = formulaInyecionInicial(REF_C2H4,CHAMBER_VOLUMEN); 
          
          
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 4);
          _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
          
          desiredEthyleneFlowRate += baseEthyleneFlowRate; 
          
          //---------------
          
          //Esto genera una orden para que se imprime en el debug
          //Serial.print("Modo Desverdizacion :"); Serial.println(estadoModoDesverdizacion);
          debugEstadoModoDesverdizacion = MODO_DESVERDIZACION;
          
         
        }
        
        switch (estadoModoDesverdizacion)
        {
        case MODO_INYECCION_INICIAL:          
          
          //Serial.print("desiredEthyleneFlowRate :"); Serial.println(String(desiredEthyleneFlowRate, 4));
          if(getTimerIntEthyleneFlow()==0){
            estadoModoDesverdizacion = MODO_INYECCION_MANTENIMIENTO;
            //Iniciamos inyeccion por mantenimiento
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 2); 
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 4);
            //Serial.print("Pasa a Modo Desverdizacion :"); Serial.println(estadoModoDesverdizacion);
          }

         
          //Esto genera una orden para que se imprime en el debug
           //Serial.println("Ejecutando MODO_INYECCION_INICIAL");
          debugEstadoModoDesverdizacion = MODO_INYECCION_INICIAL; 
          
          break;

        case MODO_INYECCION_MANTENIMIENTO:
        
          
          //Esto genera una orden para que se imprime en el debug
          //Serial.println("Ejecutando MODO_INYECCION_MANTENIMIENTO");
          debugEstadoModoDesverdizacion = MODO_INYECCION_MANTENIMIENTO; 

          if(INICIO_CICLO_MODO_MANTENIMIENTO){
            
            if(!START2)
            {
              
              CLEAR_FINAL_INJECTION_MESSAGE_ACTIVATED;
              desiredEthyleneFlowRate = 0;
              CLEAR_FANOUT_ACTIVATED;
              inyeccionPorPuerta01();
              inyeccionPorPuerta02();
              inyeccionPorVentiladoresCO2();
              inyeccionPorFuga();
            }
            
          }
          break;
        
        default:
          break;
        }
        //Aplico PID
        valueEthyleneFlowSetpointNormalization = desiredEthyleneFlowRate / CONST_NORMALIZATION_ETHYLENE_FLOW_PID; 
        //Serial.println("---------------------------------------------");
        //Serial.print("Entrada valueEthyleneFlowNormalization : "); Serial.println(valueEthyleneFlowNormalization);
        //Serial.print("valueEthyleneFlowSetpointNormalization  :"); Serial.println(valueEthyleneFlowSetpointNormalization);
        
        
        ethyleneFlowPID->SetMode(AUTOMATIC);
        ethyleneFlowPID->SetTunings(KP_ETHYLENE_FLOW_PID,KI_ETHYLENE_FLOW_PID,KD_ETHYLENE_FLOW_PID);
        ethyleneFlowPID->Compute();
        analogOutputModule1Values[2] = ethyleneFlowPIDOutput;

        ValidationPIDFlowethylene.valor = ethyleneFlowPIDOutput;

        ValidationPID(&ValidationPIDFlowethylene,ETHYLENE__FLOW_PID_LIMIT_MIN);
        if (ValidationPIDFlowethylene.flag)
        {
          analogOutputModule1Values[2] = ETHYLENE__FLOW_PID_LIMIT_CLOSE;
        }

         //------Funcion que ejecuta si esta activo el debuger----------
        debugControlEthyleneFlow();
        //------------------------------------------------------------

        
        
      }
      //else{
        //----------------------------
      //  ethyleneFlowPID->SetMode(MANUAL);
      //  if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4))
      //  {
      //    analogOutputModule1Values[2] = ethylenePIDLimitMinClose;
      //  }
      //} 
}


/**
 * @brief Inyeccion por puerta 01
*/
void Chamber::inyeccionPorPuerta01(){
         /////Inyección por puerta 1 abierta con detección de flanco de subida/////
        if (VALUE_DOOR_1)
        {
          // si hay un flanco de subida
          if (gateRaise1)
          {
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 6);
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 5);   
            //*ethyleneInyectionTimesPointer -= _modbusTCPServer->holdingRegisterRead(addressOffset + 62); //tiempo de inyeccion deteccion de puertas
            //*(ethyleneInyectionTimesPointer + 1) += _modbusTCPServer->holdingRegisterRead(addressOffset + 62);
            setTimerOnDoor01EthyleneFlow(_modbusTCPServer->holdingRegisterRead(addressOffset + 62));
            gateRaise1 = 0;
            //falta una bandera
            Serial.println("Inyeccion Puerta 1 activado");
          }
        }
        else
        {
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 6);
          gateRaise1 = 1;
        }

        if(getTimerOnDoor01EthyleneFlow()>0){
          double baseEthyleneFlowRate = (1.5 * _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) *
                                          _modbusTCPServer->holdingRegisterRead(addressOffset + 10)) / 1500;

          desiredEthyleneFlowRate += baseEthyleneFlowRate; // asigno mi nuevo setpoint
          _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);

          //
          Serial.print("tiempo de inyeccion 10s p1 :"); Serial.println(getTimerOnDoor01EthyleneFlow());
          Serial.print("baseEthyleneFlowRate  p1 :"); Serial.println(baseEthyleneFlowRate);
          Serial.print("desiredEthyleneFlowRate p1 :"); Serial.println(desiredEthyleneFlowRate);
        }   
}

/**
 * @brief Inyeccion por puerta 02
*/
void Chamber::inyeccionPorPuerta02(){
  /////Inyección por puerta 2 abierta con detección de flanco de subida/////
        if (VALUE_DOOR_2)
        {
          if (gateRaise2)
          {
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 7);
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 6);
           // ethyleneInyectionTimesPointer -= _modbusTCPServer->holdingRegisterRead(addressOffset + 62);//tiempo de inyeccion deteccion de puertas
           // (ethyleneInyectionTimesPointer + 1) += _modbusTCPServer->holdingRegisterRead(addressOffset + 62);
            setTimerOnDoor02EthyleneFlow(_modbusTCPServer->holdingRegisterRead(addressOffset + 62));
            gateRaise2 = 0;
            Serial.println("Inyeccion Puerta 2 activado");
          }
        }
        else
        {
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 7);
          gateRaise2 = 1;
        }
        if(getTimerOnDoor02EthyleneFlow()>0){
          double baseEthyleneFlowRate = (1.5 * _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) *
                                          _modbusTCPServer->holdingRegisterRead(addressOffset + 10)) / 1500;

            desiredEthyleneFlowRate += baseEthyleneFlowRate; // asigno mi nuevo setpoint
            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);

          //
          Serial.print("tiempo de inyeccion 10s p2 :"); Serial.println(getTimerOnDoor02EthyleneFlow());
          Serial.print("baseEthyleneFlowRate  p2 :"); Serial.println(baseEthyleneFlowRate);
          Serial.print("desiredEthyleneFlowRate p2 :"); Serial.println(desiredEthyleneFlowRate);
        }  
}

/**
 * @brief Inyeccion por los ventiladores de CO2
*/
void Chamber::inyeccionPorVentiladoresCO2(){
    // C2H4 control selection && ethyleneFlowRateCO2ControlGasesBalance
      if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4))
      {
        //-----
        if (analogOutputModule1Values[0] > PID_CO2_CONTROL_MIN) //Iniciamos la inyección de etileno si
        { //tenemos los ventiladores en marcha
          ethyleneFlowRateCO2Control = 1;
        }
        else
        { //no tenemos los ventiladores en marcha
          ethyleneFlowRateCO2Control = 0;
        }

        if (ethyleneFlowRateCO2Control)
        {
        //-----
          double airflow = ((double)analogOutputModule1Values[0] +
                          (double)analogOutputModule1Values[1]) *
                          (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 136);

          //Calculo para la reglas de las mezclas
          double ratioConcentration = _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) /
                                    _modbusTCPServer->holdingRegisterReadLong(addressOffset + 12);

          double extractionEthyleneFlowRate = ((airflow * ratioConcentration) / (1 - ratioConcentration));
          desiredEthyleneFlowRate += extractionEthyleneFlowRate; //lo añado al setpoint
          _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
          
          //
          Serial.print("inyeccion por ventiladores co2 :"); Serial.println(analogOutputModule1Values[0]> PID_CO2_CONTROL_MIN);
          Serial.print("extractionEthyleneFlowRate :"); Serial.println(extractionEthyleneFlowRate);
          Serial.print("desiredEthyleneFlowRate :"); Serial.println(String(desiredEthyleneFlowRate, 4));
          
        }
        
      }
}

/**
 * @brief Inyeccion por fugas y perdidas
*/
void Chamber::inyeccionPorFuga(){
  //Inyección mantenimiento por fugas
  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 2) && (!flagInyeccionPorFuga)) //Si inicio ciclo actico
  {
    
    setTimerOnFugaEthyleneFlow(_modbusTCPServer->holdingRegisterRead(addressOffset + 60));
    setTimerInyeccionFugaEthyleneFlow(_modbusTCPServer->holdingRegisterRead(addressOffset + 61));
    flagInyeccionPorFuga = 1;
    flagIntervaloInyeccionPorFuga = 1;
  }

  //tiempo de inyeccion
  if(getTimerOnFugaEthyleneFlow()==0 &&flagIntervaloInyeccionPorFuga){

    
    if(getTimerInyeccionFugaEthyleneFlow()>0){
      //Aplico formula
      
      double baseEthyleneFlowRate = (1.5 * _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) *
                                          _modbusTCPServer->holdingRegisterRead(addressOffset + 10)) / 1500;
      desiredEthyleneFlowRate += baseEthyleneFlowRate; // asigno mi nuevo setpoint
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);

      //
      Serial.print("inyeccion por fuga :"); Serial.println(getTimerInyeccionFugaEthyleneFlow());
      Serial.print("baseEthyleneFlowRate :"); Serial.println(baseEthyleneFlowRate);
      Serial.print("desiredEthyleneFlowRate :"); Serial.println(String(desiredEthyleneFlowRate, 4));

    }
    else
    {
      setTimerInyeccionFugaEthyleneFlow(0);
      flagInyeccionPorFuga = 0;
      flagIntervaloInyeccionPorFuga = 0;
    }
  }
}


/**
 * @brief modoConservacion
*/
void Chamber::modoConservacion()
{

      ethylenePID->SetMode(MANUAL);
      analogOutputModule1Values[2] = 4000;
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 4);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);

      //Serial.println("modo conservacion");
      
}
/** @brief enableControl verifica los estado del holdregister, para activar el sistema de control de Humedad, Co2 y Ethyleno al maximo
*/

//----> Tarea 2
void Chamber::enableControl(){
  
  //ENABLE TEMPERATURE EXTERNAL
  if (ENABLE_CONTROL_TEMPERATURE_EXTERNAL)
  {
    flagEnableControlSystemTemperatureExternal = 1;
  }
  else
  {
    flagEnableControlSystemTemperatureExternal = 0;
  }

  //ENABLE CONTROL TEMPERATURE AEROTERMO
  if (ENABLE_CONTROL_TEMPERATURE_AEROTERMO)
  {
    flagEnableControlSystemTemperatureAerotermo = 1;
  }
  else
  {
    flagEnableControlSystemTemperatureAerotermo = 0;
  }

  //ENABLE ETHYLENE
  if (ENABLE_ETHYLENE)
  {
    flagEnableControlSystemEthylene = 1;
  }
  else
  {
    flagEnableControlSystemEthylene = 0;
  }

  //Enable HUMIDITY
  if (ENABLE_CONTROL_HUMIDITY)
  {
    flagEnableControlSystemHumidity = 1;
  }
  else
  {
    flagEnableControlSystemHumidity = 0;
  }
  

  //Enable CO2
  if (ENABLE_CONTROL_CO2)
  {
    flagEnableControlSystemCO2 = 1;
  }
  else
  {
    flagEnableControlSystemCO2 = 0;
  }

  //Enable CO2 OnOff
  if (ENABLE_CONTROL_CO2_ON_OFF)
  {
    flagEnableControlSystemCO2OnOff = 1;
  }
  else
  {
    flagEnableControlSystemCO2OnOff = 0;
  }

  //Enable CO2 PID
  if (ENABLE_CONTROL_CO2_PID)
  {
    flagEnableControlSystemCO2Pid = 1;
  }
  else
  {
    flagEnableControlSystemCO2Pid = 0;
  }
  

  //ENABLE ETHYLENE
  if (ENABLE_ETHYLENE)
  {
    flagEnableControlSystemEthylene = 1;
  }
  else
  {
    flagEnableControlSystemEthylene = 0;
  }

  //ENABLE ETHYLENE FLOW
  if (ENABLE_ETHYLENE_FLOW)
  {
    flagEnableControlSystemEthyleneFlow = 1;
  }
  else
  {
    flagEnableControlSystemEthyleneFlow = 0;
  }

    
  
  

}

/**
 * @brief enableInputOutput verifica los estado del holdregister, para habilitar las entradas y salidas del sistema
*/
//----> Tarea 2
void Chamber::enableInputOutput(){

  if(!ENABLE_SAFETY_RELAY_RESET){
    digitalWrite(SAFETY_RELAY_RESET, LOW);
  }

  if(!ENABLE_INPUT_FAN_1){
    //------------------------
    digitalWrite(INPUT_FAN_1, LOW);
    indicadoresEstados.inputFan1 = 0;
    //------------------------
    analogOutputModule1Values[0] = 4000;
  }

  if(!ENABLE_OUTPUT_FAN_1){
    //------------------------
        digitalWrite(OUTPUT_FAN_1, LOW);
        indicadoresEstados.outputFan1 = 0;
        //------------------------
    analogOutputModule1Values[0] = 4000;
  }

  if(!ENABLE_INPUT_FAN_2){
    //------------------------
        digitalWrite(INPUT_FAN_2, LOW);
        indicadoresEstados.inputFan2 = 0;
        //------------------------
    analogOutputModule1Values[1] = 4000;
  }

  if(!ENABLE_OUTPUT_FAN_2){
    //------------------------
        digitalWrite(OUTPUT_FAN_2, LOW);
        indicadoresEstados.outputFan2 = 0;
        //------------------------
    analogOutputModule1Values[1] = 4000;
  }

  if(!ENABLE_AEROHEATERS){
    digitalWrite(AEROHEATERS, LOW);
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 259);
  }

  if(!ENABLE_HUMIDITY_WATER_VALVES){
    digitalWrite(HUMIDITY_WATER_VALVES, LOW);
  }

  if(!ENABLE_HUMIDITY_AIR_VALVES){
    
  }

  if(!ENABLE_COOLING_REQUEST){
    digitalWrite(COOLING_REQUEST, LOW);
  }

  if(!ENABLE_HEATING_REQUEST){
    digitalWrite(HEATING_REQUEST, LOW);
  }

  if(!ENABLE_CONTROL_COOLING_REQUEST){
    digitalWrite(CONTROL_COOLING_REQUEST, LOW);
  }

  if(!ENABLE_CONTROL_HEATING_REQUEST){
    digitalWrite(CONTROL_HEATING_REQUEST, LOW);
  }

  if(!ENABLE_EVAPORATOR_FAN_ACTIVATOR){
    digitalWrite(EVAPORATOR_FAN_ACTIVATOR, LOW);
  }

  if(!ENABLE_ALARM_SET){
    digitalWrite(ALARM_SET, LOW);
  }

  if (!ENABLE_AUTOTEL_SELECTOR_HR)
  {
    autoSelectorValue = 0;
  }
  else
  {
    autoSelectorValue = 1;
  }

  // StateAutoTelSelector
  this->stateAutoTelSelector();
  


}



/**
 * @brief forcedControl verifica los estado del holdregister, para forza las salidas de Humedad, Co2 y Ethyleno al maximo
*/
//----> Tarea 3
void Chamber::forcedControl()
{

  if(FORCED_SAFETY_RELAY_RESET){
      //condition
      digitalWrite(SAFETY_RELAY_RESET, HIGH);
  }
  else
  {
      digitalWrite(SAFETY_RELAY_RESET, LOW);
  }

  if (FORCED_INPUT_FAN_1)
  {
    //------------------------
    digitalWrite(INPUT_FAN_1, HIGH);
    indicadoresEstados.inputFan1 = 1;
    //------------------------
    analogOutputModule1Values[0]=20000;
    
  }

  if (FORCED_INPUT_FAN_2)
  {
    //------------------------
        digitalWrite(INPUT_FAN_2, HIGH);
        indicadoresEstados.inputFan2 = 1;
        //------------------------
    analogOutputModule1Values[0]=20000;
  }

  if (FORCED_OUTPUT_FAN_1)
  {
    //------------------------
        digitalWrite(OUTPUT_FAN_1, HIGH);
        indicadoresEstados.outputFan1 = 1;
        //------------------------
    analogOutputModule1Values[1]=20000;
  }
  
  if (FORCED_OUTPUT_FAN_2)
  {
    //------------------------
        digitalWrite(OUTPUT_FAN_2, HIGH);
        indicadoresEstados.outputFan2 = 1;
        //------------------------
    analogOutputModule1Values[1]=20000;
  }

  if (FORCED_AEROHEATERS)
  {
    //condition
    digitalWrite(AEROHEATERS, HIGH);
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 259);
  }
  
  if (FORCED_HUMIDITY_WATER_VALVES)
  {
    //condition
    digitalWrite(HUMIDITY_WATER_VALVES, HIGH);
  }

  if(FORCED_COOLING_REQUEST){
    //condition
    digitalWrite(COOLING_REQUEST, HIGH);
  }  
                  
  if(FORCED_HEATING_REQUEST){
    digitalWrite(HEATING_REQUEST, HIGH);
  } 

  if(FORCED_CONTROL_COOLING_REQUEST){
    digitalWrite(CONTROL_COOLING_REQUEST, HIGH);

  }   

  if(FORCED_CONTROL_HEATING_REQUEST){
    digitalWrite(CONTROL_HEATING_REQUEST, HIGH);

  }   

 //if(FORCED_EVAPORATOR_FAN_ACTIVATOR){
   //condition
  // digitalWrite(EVAPORATOR_FAN_ACTIVATOR, HIGH);
 //}  

 if(FORCED_ALARM_SET){
  //condition
  digitalWrite(ALARM_SET, HIGH);
 }
                      
 if(FORCED_ETHYLENE){
   analogOutputModule1Values[2]=20000;
 }
  
  
}

//funciones privadas para hacer debuggear en cada uno de los sistemas de control

/* Funciones privadas creadas para debugear cada una de las variables
 que se estan muestreando en la camara tal como humedad, flujo de humedad, temperatura, co2 y c2h4
*/

/* funcion para debugear el control de humedad */
void Chamber::debugControlHumidity(){
  if (debugConsole.humidity)
    {
      unsigned long timeCosoleIn = millis() - debugLastTime.humidity;
      if (timeCosoleIn > 1000) //para que se imprima cada 1000ms
      {
        debugLastTime.humidity = millis();
        //--------------aca se imprime todo lo que quiera
      }
    }
}

/* funcion para debugear el control de etileno */
void Chamber::debugControlEthylene(){

  if (debugConsole.ethylene)
    {
      unsigned long timeCosoleIn = millis() - debugLastTime.ethylene;
      if (timeCosoleIn > 1000) //para que se imprima cada 1000ms
      {
        debugLastTime.ethylene = millis();
        //--------------aca se imprime todo lo que quiera
        Serial.println("-------Console Ethylene-----------------------------------------");
        Serial.print("ethyleneSetpoint : "); Serial.println(ethyleneSetpoint);
        Serial.print("Entrada Sensor C2H4 : "); Serial.println(calculatedSensorValues[2]);
        Serial.print("Salida PID ethyleneControl: "); Serial.println(analogOutputModule1Values[2]);
        Serial.println("-------________________-----------------------------------------");
      }
    }
}

/* funcion para debugear el control del flujo de etileno */
void Chamber::debugControlEthyleneFlow(){
  if (debugConsole.ethyleneflow)
  {
    unsigned long timeConsoleIn = millis();
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
      
      switch (debugEstadoModoDesverdizacion)
      {
      case MODO_INYECCION_INICIAL:
        Serial.println("Ejecutando MODO_INYECCION_INICIAL");
        break;
      
      case MODO_INYECCION_MANTENIMIENTO:
        Serial.println("Ejecutando MODO_INYECCION_MANTENIMIENTO");
        break;
      
      case MODO_DESVERDIZACION:
        Serial.print("Modo Desverdizacion :"); Serial.println(estadoModoDesverdizacion);
        break;
      
      default:
        break;
      }

      //--------------aca se imprime todo lo que quiera
      Serial.println("-------Console Ethylene Flow -------------------------------------");
      Serial.print("desiredEthyleneFlowRate  :");Serial.println(String(desiredEthyleneFlowRate, 4));
      Serial.print("Lectura Sensor Ethylene Flow  :"); Serial.println(calculatedSensorValues[4]);
      Serial.print("PID ethyleneFlowPIDOutput : ");Serial.println(ethyleneFlowPIDOutput);
      Serial.println("-------________________------------------------------------------");
    }    
  }
}

/* funcion para debugear el control de co2 */
void Chamber::debugControlCo2(){
  if (debugConsole.co2)
  {
    unsigned long timeConsoleIn = millis();
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
      //--------------aca se imprime todo lo que quiera
      Serial.println("-------Console CO2 -------------------------------------");
      Serial.print("ValidationPIDCO2.flag : "); Serial.println(ValidationPIDCO2.flag);
      Serial.print("CO2_Setpoint : "); Serial.println(CO2Setpoint);
      Serial.print("Sensor CO2 V1 : "); Serial.println(analogOutputModule1Values[0] );
      Serial.print("Sensor CO2 V2 : "); Serial.println(analogOutputModule1Values[1] );
      Serial.print("Salida PID CO2_Control: "); Serial.println(CO2PIDOutput);
      Serial.println("-------________________--------------------------------");
    }   
  }
}

/* funcion para debugear el control de temperatura */
void Chamber::debugControlTemp(){
  if (debugConsole.temperature)
  {
    unsigned long timeConsoleIn = millis();
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
      //--------------aca se imprime todo lo que quiera
      Serial.println("-------Console temperature -----------------------------");
      Serial.print("Sensor temperature value : "); Serial.println(calculatedSensorValues[0]);
      Serial.print("Threshold heating activator : "); Serial.println(_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) -
                                                                     _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 22)) &&
                                                                     digitalRead(EXT_HEATER_AVAILABLE)));
      Serial.print("Heating activation : "); Serial.println(HEATING_REQUEST);
      Serial.print("Threshold cooling activator : "); Serial.println(_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 16) +
                                                                     _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 24)) &&
                                                                     digitalRead(EXT_COOLER_AVAILABLE))
      Serial.print("Cooling activation : "); Serial.println(COOLING_REQUEST);       
      Serial.println("-------________________---------------------------------");
    }
  }
}

//Zeta de emergencia
//-----> Tarea 4
void setaEmergency(){
  if (digitalRead(EMERGENCY_STOP)){
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 0);
  } 
  else{
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 0);
  }
}


//micro cortes
//------>Tarea 5
void Chamber::atiendeMicroCutsInterrup(unsigend char *mflagMicroCutsPointer){
  
  if (*mflagMicroCutsPointer)
  {
    if (digitalRead(MICRO_CUTS_DETECTION))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 3);
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 3);
    }
      *mflagMicroCutsPointer = 0;
    }
  
  
}

//---------> Tarea 5

void Chamber::atiendeGeneralSwitchDetect(void){

  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 3)){
    if(digitalRead(GENERAL_SWITCH_DETEC)){
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 0);
      // se limpia el 0,3
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 3);
    }
    else{

      if (timerMicroCut.flag10 == 0)
      {
        timerMicroCut.timer10 = 10;
        timerMicroCut.flag10 = 1;
      }
      else
      {
        if (timerMicroCut.timer10 == 0)
        {
          if (timerMicroCut.flag03 == 0)
          {
            timerMicroCut.timer03 = 3;
            timerMicroCut.flag03 = 1;
          }
          else
          {
            if(timerMicroCut.timer03 > 0){
              digitalWrite(SAFETY_RELAY_RESET,HIGH);
            }
            else
            {
              digitalWrite(SAFETY_RELAY_RESET,LOW);
              timerMicroCut = {0,0,0,0};
              // se limpia el 0,3
              _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 3);
            }
          }
        }
        
      }

}

//---------> Tarea 5
void Chamber::setupSafetyRelayReset(void){
  if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 3))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 0, 3);
    digitalWrite(SAFETY_RELAY_RESET, HIGH);
    delay(1000);
    digitalWrite(SAFETY_RELAY_RESET,LOW);
	  _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 3);
    
  }


// StateAutoTelSelector ---> Tarea 11
void Chamber::stateAutoTelSelector(void){

  
  

  if (digitalRead(AUTO_TEL_SELECTOR)) 
  {
    //Asignamos el HR a 1
    SET_AUTOTEL_SELECTOR_HR;

    if (AUTO_TEL_SELECTOR_STATE){
      // control humedad y temperatura  bloqueado
      autoSelectorValue = 0;

      if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0)){
        SET_AUTOTEL_SELECTOR_HR; //ENABLE_AUTOTEL_SELECTOR_HR = 1;
      }
      else
      {
        CLEAR_AUTOTEL_SELECTOR_HR; //ENABLE_AUTOTEL_SELECTOR_HR = 0;
      }
    }
    else
    {
      // control humedad y temperatura  desbloqueado
      autoSelectorValue = 1;
      CLEAR_AUTOTEL_SELECTOR_HR;
    }
  }
  else {
    CLEAR_AUTOTEL_SELECTOR_HR;
  }

}

Chamber::~Chamber(){
  
}
