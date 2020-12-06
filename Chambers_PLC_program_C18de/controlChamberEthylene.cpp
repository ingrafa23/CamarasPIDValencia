#include "controlChamberEthylene.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"
#include "mapsensor.h"


void controlChamberEthylene::setup(){

}

controlChamberEthylene::controlChamberEthylene(ModbusTCPServer *modbusTCPServer,int maddressOffset)
{
  _modbusTCPServer = modbusTCPServer;
  addressOffset = maddressOffset;

//-----------------
int mConts =CONST_NORMALIZATION_ETHYLENE_PID;
  _mapsensor = new mapsensor(_modbusTCPServer,
                        addressOffset + 262,            // C2h4 Measure
                        addressOffset + 83,             // LowLimit1
                        addressOffset + 84,             // HighLimit1
                        addressOffset + 85,             // zeroSensor1
                        addressOffset + 86,             // spanSensor1
                        mConts);   // constante de normalización co2
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

void controlChamberEthylene::alarm(){
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


void controlChamberEthylene::readEthylene(double medidaSensor){
  _mapsensor->mapFloatMeasurementSensor(medidaSensor);
  valueEthyleneNormalization = _mapsensor->getValueSensorNormaliced();
  calculatedSensorValues = _mapsensor->getValueSensor();
}


void controlChamberEthylene::ethyleneControl()
{
    ////////////////////Modo desverdizado activo///////////////////////
    //Modo trabajo desverdización
    if ( _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) ) 
            //Cámara en marcha
    {
      /////////////////Bloque control por PID//////////////////////////
      if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1) &&
          _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4))
      {
        if(alarmSensorEthyl1 == false && alarmSensorEthyl2 == false) //Control por análisis activo
        {
          float relativeError = (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) -
                                _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 262)) /
                                _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18);

          if (relativeError >= (1 - (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 140) / 100)))
          { 
            ethylenePID->SetMode(MANUAL);

            analogOutputModule1Values.value = ETHYLENE_PID_OPEN; ///----analogOutputModule1Values[2]
            analogOutputModule1Values.flag = 1;

            _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);   
          }
          else if (relativeError <= (_modbusTCPServer->holdingRegisterReadFloat(addressOffset + 140) / 100) - 1)
          {
            ethylenePID->SetMode(MANUAL);

            analogOutputModule1Values.value = ETHYLENE_PID_CLOSE;
            analogOutputModule1Values.flag = 1;

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

            analogOutputModule1Values.value = ethylenePIDOutput;
            analogOutputModule1Values.flag = 1;

            ValidationPIDControlethylene.valor = ethylenePIDOutput;

            ValidationPID(&ValidationPIDControlethylene, ETHYLENE_PID_LIMIT_MIN);
            
            if (ValidationPIDControlethylene.flag)
            {
              analogOutputModule1Values.value = ETHYLENE_PID_CLOSE;
              analogOutputModule1Values.flag = 1;

              _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
              
            }
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

        analogOutputModule1Values.value = 4000;
        analogOutputModule1Values.flag = 1;

        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
    }
    /*END CONDITION ENABALE CONTROL SYSTEM ETHYLENE*/
}

void controlChamberEthylene::enable(){
    if (!ENABLE_ETHYLENE)
    {
        analogOutputModule1Values.value = ETHYLENE_PID_CLOSE;
        analogOutputModule1Values.flag = 1;
    }
}

void controlChamberEthylene::forced(){
    if (FORCED_ETHYLENE)
    {
        analogOutputModule1Values.value = ETHYLENE_PID_OPEN;
        analogOutputModule1Values.flag = 1;
    }
}

void controlChamberEthylene::stateIndicator(){
    if (analogOutputModule1Values.value > ETHYLENE_PID_CLOSE)
    {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
    }
    else
    {
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
    }
        
}


void controlChamberEthylene::run(double medidaSensor){
    this->readEthylene(medidaSensor);
    this->alarm(); //aqui puede haber un error pero no se
    this->ethyleneControl();
    this->enable();
    this->forced();
    this->stateIndicator();

    //------Funcion que ejecuta si esta activo el debuger----------
    //------Funcion que ejecuta si esta activo el debuger----------
        String strDebug;
        strDebug = "-------Console Ethylene Control -------------------------------------\n";
        strDebug +="Setpoint : "; 
        strDebug += String(ethyleneSetpoint,DEC);
        strDebug = "\n";
        strDebug +="Sensor Input  : "; 
        strDebug += String(calculatedSensorValues,DEC);
        strDebug = "\n";
        strDebug +="Valor PID: "; 
        strDebug += String(analogOutputModule1Values.value,DEC);
        strDebug = "\n";
        strDebug = "--------------------------------------------\n";
        this->debugControlEthylene(strDebug);
    //----------------
}


bool controlChamberEthylene::getAlarmOnGeneral(){
    return alarmOnGeneral;
}

int controlChamberEthylene::getAnalogOutputModule1ValuesEthylene(){
    return analogOutputModule1Values.value;
}

bool controlChamberEthylene::getAnalogOutputModule1FlagEthylene(){
    bool m_resp = analogOutputModule1Values.flag;
    analogOutputModule1Values.flag = 0; // indicar que fue atendida
    return m_resp;
}

//funciones privada
/* funcion para debugear el control */
void controlChamberEthylene::debugControlEthylene(String mdebug){
  if (debugConsole.ethylene)
  {
    unsigned long timeConsoleIn = abs(debugLastTime.ethylene -  millis());
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
    debugLastTime.ethylene = millis();
      //--------------aca se imprime todo lo que quiera
      Serial.println(mdebug);
    }   
  }
}



controlChamberEthylene::~controlChamberEthylene(){

}

