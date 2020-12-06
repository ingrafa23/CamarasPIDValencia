#include "controlChambers.h"
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
//---------------------------------------------------
_mapsensorReadOutputFan1 = new mapsensor(&modbusTCPServer,
                        addressOffset + 285,            // Measure
                        addressOffset + 119,             // LowLimit1
                        addressOffset + 120,             // HighLimit1
                        addressOffset + 121,             // zeroSensor1
                        addressOffset + 121,             // spanSensor1
                        1);   // constante de normalización 
//----------------------------------------
_mapsensorReadOutputFan2 = new mapsensor(&modbusTCPServer,
                        addressOffset + 289,            // Measure
                        addressOffset + 123,             // LowLimit1
                        addressOffset + 124,             // HighLimit1
                        addressOffset + 125,             // zeroSensor1
                        addressOffset + 126,             // spanSensor1
                        1);   // constante de normalización
//----------------------------------------

  readsensorInput = new readsensor(&_modbusTCPClient1);

  _controlchamberco2 = new controlchamberco2(&modbusTCPServer,addressOffset);
  _controlchambershumidity = new controlchambershumidity(&modbusTCPServer,addressOffset);
  _controlChamberEthylene = new controlChamberEthylene(&modbusTCPServer,addressOffset);
  _controlChamberEthyleneFlow = new controlChamberEthyleneFlow(&modbusTCPServer,addressOffset);
  _controlChamberTemperature = new controlChamberTemperature(&modbusTCPServer,addressOffset);
  
}

Chamber::init()
{
  
  //inicializo los timer de alamr a de puertas y ventilador 

  timerAlarmNoVentilation = hour2seg(MAX_TIME_ALARM_NO_VENTILATION);
  timerOpenDoorTimeAlarm1 = MAX_TIME_OPEN_DOOR_1;
  timerOpenDoorTimeAlarm2 = MAX_TIME_OPEN_DOOR_2;

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

//este es uno nuevo no borrar

void Chamber::run(){
//Lectura de todos los sensores Analogicos de entrada
  readsensorInput->runReadSesor();

  //falta alarmas

  //run del control CO2
  _controlchamberco2->run(readsensorInput->getValueSensor(rawValueInputModule1Co2));
  //run del control humidity
  _controlchambershumidity->run(readsensorInput->getValueSensor(rawValueInputModule1Hum),autoSelectorValue);
  // run del control de ehtylene
  _controlChamberEthylene->run(readsensorInput->getValueSensor(rawValueInputModule1Ethylene));
  // run control de ethylene flow
  _controlChamberEthyleneFlow->run(analogRead(ETHYLENE_FLOW_IN),_controlchamberco2->getAnalogOutputModule1ValuesCo2(0),
                              _controlchamberco2->getAnalogOutputModule1ValuesCo2(1),_controlchamberco2->getMinCo2());
  // run control de temperatura
  _controlChamberTemperature->run(readsensorInput->getValueSensor(rawValueInputModule1Temp),autoSelectorValue);                         

  //habilitadores
  this->enable();
  //forzados
  this->forced();
  // escritura de entradas
  //this->measurements();
  // escrituras de salidas
  this->writeAnalogValues();
  this->writeIODigital();
  //indicadores
  this->stateIndicator();

  //stateAutoTelSelector
  this->stateAutoTelSelector();

}

void Chamber::stateIndicator(){
  if (digitalRead(ALARM_SET))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 339, 4);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 339, 4);
  }
  //-------------------------
  if (digitalRead(EVAPORATOR_FAN_ACTIVATOR))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 339, 1);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 339, 1);
  }
  //----------------------------------------------------------------
  if (digitalRead(SAFETY_RELAY_RESET))
  {
    //_modbusTCPServer->holdingRegisterSetBit(addressOffset + 339, xxx);
  }
  else
  {
    //_modbusTCPServer->holdingRegisterClearBit(addressOffset + 339, xxx);
  }
  
  //-----------
  if (autoSelectorValue)
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 339, 3);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 339, 3);
  }
  
  //----------------------------------------------
  if (digitalRead(DOOR_1_OPEN_DETECT))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 1, 4);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 1, 4);
  }

  //-----------------------------
  if (digitalRead(DOOR_2_OPEN_DETECT))
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 1, 5);
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 1, 5);
  }
}

void Chamber::writeAnalogValues()
{
  analogOutputModule1Values[analogOutputModule1ValuesCo2_0] = _controlchamberco2->getAnalogOutputModule1ValuesCo2(0);
  analogOutputModule1Values[analogOutputModule1ValuesCo2_0] = _controlchamberco2->getAnalogOutputModule1ValuesCo2(1);


  if (_controlChamberEthylene->getAnalogOutputModule1FlagEthylene())
  {
    analogOutputModule1Values[analogOutputModule1ValuesEthylene] = _controlChamberEthylene->getAnalogOutputModule1ValuesEthylene();
  }else
  {
    if (_controlChamberEthyleneFlow->getAnalogOutputModule1FlagEthyleneFlow())
    {
      analogOutputModule1Values[analogOutputModule1ValuesEthylene] = _controlChamberEthyleneFlow->getAnalogOutputModule1ValuesEthyleneFlow();
    }
    
  }
  
  
  _modbusTCPClient2->beginTransmission(HOLDING_REGISTERS, 40, 4);

  for (int i = 0; i < NumberanalogOutputModule1Values; i++)
  {
    _modbusTCPClient2->write(analogOutputModule1Values[i]);
  }

  _modbusTCPClient2->endTransmission();
}

void Chamber::writeIODigital(){
  digitalWrite(ALARM_SET,controlChambersIO.alarmSet);
  digitalWrite(EVAPORATOR_FAN_ACTIVATOR,controlChambersIO.evaporatorFanActivator);
  digitalWrite(SAFETY_RELAY_RESET,controlChambersIO.safetyRelayReset);
}

Chamber::alarms()
{
  //Alarma General
  if(alarmOn == true || _controlchamberco2->getAlarmOnGeneral() == true || _controlchambershumidity->getAlarmOnGeneral() == true
    || _controlChamberEthylene->getAlarmOnGeneral == true || _controlChamberTemperature->getAlarmOnGeneral() ==true)
  {
    controlChambersIO.alarmSet = 1; //----- digitalWrite(ALARM_SET, HIGH);
  }
  else
  {
    controlChambersIO.alarmSet = 0; //----- digitalWrite(ALARM_SET, LOW);
  }

  
  //EMPIEZAN VERIFICAR ALARMAS NO VENTILAR 
  if (timerAlarmNoVentilation > 0) {
  if( (VALUE_ACTUAL_INPUT_FAN1) || (VALUE_ACTUAL_OUTPUT_FAN1) || (VALUE_ACTUAL_INPUT_FAN2) || (VALUE_ACTUAL_OUTPUT_FAN2) ) {

    //unsigned int maxTimeAlarmVentilation = EEPROM.read(142)<<8 + EEPROM.read(143);
    timerAlarmNoVentilation = hour2seg(MAX_TIME_ALARM_NO_VENTILATION);
    flagTimerAlarmNoVentilationPointer = false;
  }
  else
  {
    if(timerAlarmNoVentilation == 0 ){
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
    if (timerOpenDoorTimeAlarm1 == 0){
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
    timerOpenDoorTimeAlarm1 = MAX_TIME_OPEN_DOOR_1;
  } 

  // PUERTA 2
  if (VALUE_DOOR_2)
  {
    if (timerOpenDoorTimeAlarm2 == 0){
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
    timerOpenDoorTimeAlarm2 = MAX_TIME_OPEN_DOOR_2;
  } 
  //FINALIZAN VERIFICACION ALARMA POR TIEMPO DE PUERTA ABIERTA

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




void Chamber::readOutputFan1(double medidaSensor){
  _mapsensorReadOutputFan1->mapFloatMeasurementSensor(medidaSensor);
  calculatedSensorValuesOutputFan1 = _mapsensorReadOutputFan1->getValueSensor(); //------------calculatedSensorValues[SensorOutputFan1ValuePos]
}


void Chamber::readOutputFan2(double medidaSensor){
  _mapsensorReadOutputFan2->mapFloatMeasurementSensor(medidaSensor);
  calculatedSensorValuesOutputFan2 = _mapsensorReadOutputFan2->getValueSensor(); //------------calculatedSensorValues[SensorOutputFan2ValuePos]
}


Chamber::measurements()
{
  readOutputFan1(analogRead(OUTPUT_FAN_PHASE_1));
  readOutputFan2(analogRead(OUT_FAN_PHASE_2));
}



/**
 * @brief enableInputOutput verifica los estado del holdregister, para habilitar las entradas y salidas del sistema
*/
//----> Tarea 2
void Chamber::enable(){

  if (!ENABLE_SAFETY_RELAY_RESET)
  {
    controlChambersIO.safetyRelayReset = 0;
  }

  if(!ENABLE_EVAPORATOR_FAN_ACTIVATOR){
    controlChambersIO.evaporatorFanActivator = 0; //-------digitalWrite(EVAPORATOR_FAN_ACTIVATOR, LOW);
  }

  if(!ENABLE_ALARM_SET){
    controlChambersIO.alarmSet = 0; //----- digitalWrite(ALARM_SET, LOW);
  }

  if (!ENABLE_AUTOTEL_SELECTOR_HR)
  {
    autoSelectorValue = 0;
  }
  else
  {
    autoSelectorValue = 1;
  }

}



/**
 * @brief forcedControl verifica los estado del holdregister, para forza las salidas de Humedad, Co2 y Ethyleno al maximo
*/
//----> Tarea 3
void Chamber::forced()
{
  if (FORCED_SAFETY_RELAY_RESET)
  {
    controlChambersIO.safetyRelayReset = 1;
  }

  if(FORCED_EVAPORATOR_FAN_ACTIVATOR)
  {
    controlChambersIO.evaporatorFanActivator = 1; //-------digitalWrite(EVAPORATOR_FAN_ACTIVATOR, LOW);
  }

  if(FORCED_ALARM_SET){
    controlChambersIO.alarmSet = 1; //----- digitalWrite(ALARM_SET, LOW);
  }

  if (FORCED_AUTOTEL_SELECTOR_HR)
  {
    autoSelectorValue = 1;
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

//-----> Tarea 5
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
              
              controlChambersIO.safetyRelayReset = 1; //------digitalWrite(SAFETY_RELAY_RESET,HIGH);
            }
            else
            {
              
              controlChambersIO.safetyRelayReset = 0; //-----digitalWrite(SAFETY_RELAY_RESET,LOW);
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
    controlChambersIO.safetyRelayReset = 1; //------digitalWrite(SAFETY_RELAY_RESET,HIGH);
    delay(1000);
    controlChambersIO.safetyRelayReset = 0; //-----digitalWrite(SAFETY_RELAY_RESET,LOW);
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
