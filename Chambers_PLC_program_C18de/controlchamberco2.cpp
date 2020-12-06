#include "controlchamberco2.h"
#include "constPID.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"
#include "mapsensor.h"


struct StructValidationPID  ValidationPIDCO2;
 
void controlchamberco2::setup(){

}

controlchamberco2::controlchamberco2(ModbusTCPServer *modbusTCPServer,int maddressOffset)
{
 
  _modbusTCPServer = modbusTCPServer;
  addressOffset = maddressOffset;
  //-----------------
  _mapsensor = new mapsensor(&modbusTCPServer,
                        addressOffset + 264,            // CO2 Measure
                        addressOffset + 87,             // LowLimit1
                        addressOffset + 88,             // HighLimit1
                        addressOffset + 89,             // zeroSensor1
                        addressOffset + 90,             // spanSensor1
                        CONST_NORMALIZATION_CO2_PID);   // constante de normalización co2
  //-----------------

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

}

//alarmas co2
controlchamberco2::alarm(){
    //EMPIEZAN ALARMAS CO2

  if (calculatedSensorValues > _modbusTCPServer->holdingRegisterRead(addressOffset + 42)) // > setpoint alarm co2
  {
    if (timerLimitAlarmCO2 <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 3); // high co2 alarm on
      alarmOnGeneral = true;  // general alarm
      
    }
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 3); // high co2 alarm off
    timerLimitAlarmCO2 = _modbusTCPServer->holdingRegisterRead(addressOffset + 159);
  }

  //ACABAN ALARMAS CO2

  //Alarma repetición sensor de CO2

  if (calculatedSensorValues == CO2PreviusValue)
  {
    if (timerGoOffAlarmCO2 <= 0)
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 255, 2);
      alarmSensorCO21 = true;
      alarmOnGeneral = true;
    }
  }
  else
  {
    timerGoOffAlarmCO2 = _modbusTCPServer->holdingRegisterRead(addressOffset + 155);
    
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 255, 2);
    alarmSensorCO21 = false;
  }

  CO2PreviusValue = calculatedSensorValues;

//Alarma fallo sensor CO2

  if (calculatedSensorValues < _modbusTCPServer->holdingRegisterRead(addressOffset + 151) || // limite inferior fallo co2
      calculatedSensorValues > _modbusTCPServer->holdingRegisterRead(addressOffset + 150))   // limite superior fallo CO2
  {
    _modbusTCPServer->holdingRegisterSetBit(addressOffset + 254, 10); // Alarma de fallo de sensor de CO2 1
    alarmSensorCO22 = true;
    alarmOnGeneral = true;
  }
  else
  {
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 254, 10); // Alarma de fallo de sensor de CO2 1
    alarmSensorCO22 = false;
  }

}




//Lectura de los Sensores de CO2
controlchamberco2::readCO2(double medidaSensor)
{
  _mapsensor->mapFloatMeasurementSensor(medidaSensor);
  valueCO2Normalization = _mapsensor->getValueSensorNormaliced();
  calculatedSensorValues = _mapsensor->getValueSensor();
}



controlchamberco2:: CO2Control(){

    //Si la cámara está en marcha regula el CO2 y si no desactiva la regulación
if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0) ||
    alarmSensorCO21 == false || alarmSensorCO22 == false )  
  {
    /////////////////////////Control de CO2 por consigna//////////////////////////
    if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 5) ) 
    {
      if (calculatedSensorValues >=
          _modbusTCPServer->holdingRegisterRead(addressOffset + 21))
      {
        analogOutputModule1ValuesCo2[0] = CO2_PID_OPEN;
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 2);
        analogOutputModule1ValuesCo2[1] = CO2_PID_OPEN;
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 3);

        controlChamberCo2IO.inputFan1 = 1;  //-----digitalWrite(INPUT_FAN_1, HIGH);
        controlChamberCo2IO.inputFan2 = 1;  //-----digitalWrite(OUTPUT_FAN_1, HIGH);
        controlChamberCo2IO.outputFan1 = 1; //-----digitalWrite(INPUT_FAN_2, HIGH);
        controlChamberCo2IO.outputFan2 = 1; //-----digitalWrite(OUTPUT_FAN_2, HIGH);
        
      }

      if (calculatedSensorValues <=
          _modbusTCPServer->holdingRegisterRead(addressOffset + 20))
      {

        analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
        analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);


        controlChamberCo2IO.inputFan1 = 0;  //-----digitalWrite(INPUT_FAN_1, LOW);
        controlChamberCo2IO.inputFan2 = 0;  //-----digitalWrite(OUTPUT_FAN_1, LOW);
        controlChamberCo2IO.outputFan1 = 0; //-----digitalWrite(INPUT_FAN_2, LOW);
        controlChamberCo2IO.outputFan2 = 0; //-----digitalWrite(OUTPUT_FAN_2, LOW);

      }
    }
    /////////////////////Final Control de CO2 por consigna////////////////////////

    //////////////////////////Control del CO2 por PID/////////////////////////////
    if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 5))
    {
      //Si la medida es menor que el límite inferior se desconecta el PID
      
      if (calculatedSensorValues <= _modbusTCPServer->holdingRegisterRead(addressOffset + 20))
      {
        CO2PID->SetMode(MANUAL);
        analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
        analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
        
        controlChamberCo2IO.inputFan1 = 0;  //-----digitalWrite(INPUT_FAN_1, LOW);
        controlChamberCo2IO.inputFan2 = 0;  //-----digitalWrite(OUTPUT_FAN_1, LOW);
        controlChamberCo2IO.outputFan1 = 0; //-----digitalWrite(INPUT_FAN_2, LOW);
        controlChamberCo2IO.outputFan2 = 0; //-----digitalWrite(OUTPUT_FAN_2, LOW);
      }
      else
      {

        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 2);
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 3);
        
        //digitalWrite(INPUT_FAN_1, HIGH); // ¿? estaba comentado ***
        controlChamberCo2IO.inputFan1 = 1;  //-----digitalWrite(INPUT_FAN_1, HIGH);
        controlChamberCo2IO.inputFan2 = 1;  //-----digitalWrite(OUTPUT_FAN_1, HIGH);
        controlChamberCo2IO.outputFan1 = 1; //-----digitalWrite(INPUT_FAN_2, HIGH);
        controlChamberCo2IO.outputFan2 = 1; //-----digitalWrite(OUTPUT_FAN_2, HIGH);

        CO2PID->SetMode(AUTOMATIC);
        CO2PID->SetTunings(KP_CO2_PID, KI_CO2_PID, KD_CO2_PID, 1);

        CO2Setpoint = (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 21);
        valueCO2SetpointNormalization = CO2Setpoint / CONST_NORMALIZATION_CO2_PID; // linea nueva
        
        CO2PID->Compute();

        analogOutputModule1ValuesCo2[0] = CO2PIDOutput;
        analogOutputModule1ValuesCo2[1] = CO2PIDOutput;

        ValidationPIDCO2.valor = CO2PIDOutput;
        ValidationPID(&ValidationPIDCO2, PID_CO2_CONTROL_MIN);
        
        

        if (ValidationPIDCO2.flag)
        {
          analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
          analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
          
        }
      }

      }
    }
    else
    {
        CO2PID->SetMode(MANUAL);
        analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
        analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
        controlChamberCo2IO.inputFan1 = 0;  //-----digitalWrite(INPUT_FAN_1, LOW);
        controlChamberCo2IO.inputFan2 = 0;  //-----digitalWrite(OUTPUT_FAN_1, LOW);
        controlChamberCo2IO.outputFan1 = 0; //-----digitalWrite(INPUT_FAN_2, LOW);
        controlChamberCo2IO.outputFan2 = 0; //-----digitalWrite(OUTPUT_FAN_2, LOW);
    }
    ///////////////////////Final Control del CO2 por PID//////////////////////////  


  }
  else
  {
    CO2PID->SetMode(MANUAL);
    ethyleneFlowRateCO2Control = 0;
    analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
    analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
    
    controlChamberCo2IO.inputFan1 = 0;  //-----digitalWrite(INPUT_FAN_1, LOW);
    controlChamberCo2IO.inputFan2 = 0;  //-----digitalWrite(OUTPUT_FAN_1, LOW);
    controlChamberCo2IO.outputFan1 = 0; //-----digitalWrite(INPUT_FAN_2, LOW);
    controlChamberCo2IO.outputFan2 = 0; //-----digitalWrite(OUTPUT_FAN_2, LOW);
  }
/*END CONDITION ENABALE CONTROL SYSTEM CO2*/

}


void controlchamberco2::enable(){
  if(!ENABLE_INPUT_FAN_1){
    controlChamberCo2IO.inputFan1 = 0;
    analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
  }

  if(!ENABLE_OUTPUT_FAN_1){
    controlChamberCo2IO.outputFan1 = 0;
    analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
  }

  if(!ENABLE_INPUT_FAN_2){
    controlChamberCo2IO.inputFan2 = 0;
    analogOutputModule1ValuesCo2[0] = CO2_PID_CLOSE;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2);
  }

  if(!ENABLE_OUTPUT_FAN_2){
    controlChamberCo2IO.outputFan2 = 0;
    analogOutputModule1ValuesCo2[1] = CO2_PID_CLOSE;
    _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3);
  }
}

void controlchamberco2::forced(){
  if (FORCED_INPUT_FAN_1)
  {
    controlChamberCo2IO.inputFan1 = 1;
    analogOutputModule1ValuesCo2[0]= CO2_PID_OPEN;
  }

  if (FORCED_OUTPUT_FAN_1)
  {
    controlChamberCo2IO.outputFan1 = 1;
    analogOutputModule1ValuesCo2[1]= CO2_PID_OPEN;
  }

  if (FORCED_INPUT_FAN_2)
  {
    controlChamberCo2IO.inputFan2 = 1;
    analogOutputModule1ValuesCo2[0]= CO2_PID_OPEN;
  }

  if (FORCED_OUTPUT_FAN_2)
  {
    controlChamberCo2IO.outputFan2 = 1;
    analogOutputModule1ValuesCo2[1]= CO2_PID_OPEN;
  }
}

void controlchamberco2::writeIO(){
  digitalWrite(INPUT_FAN_1, controlChamberCo2IO.inputFan1);
  digitalWrite(OUTPUT_FAN_1, controlChamberCo2IO.inputFan2);
  digitalWrite(INPUT_FAN_2, controlChamberCo2IO.outputFan1);
  digitalWrite(OUTPUT_FAN_2, controlChamberCo2IO.outputFan2);
}

void controlchamberco2::stateIndicator(){
  //Salida INPUT_FAN_1
    if (digitalRead(INPUT_FAN_1))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 2)
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 2)
    }

    //Salida OUTPUT_FAN_1
    if (digitalRead(OUTPUT_FAN_1))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 3)
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 3)
    }

    //Salida INPUT_FAN_2
    if (digitalRead(INPUT_FAN_2))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 12)
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 12)
    }

    //Salida OUTPUT_FAN_2
    if (digitalRead(OUTPUT_FAN_2))
    {
      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 13)
    }
    else
    {
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 13)
    }

}




void controlchamberco2::run(double medidaSendor){

    this->readCO2(medidaSendor);
    this->alarm(); //aqui puede haber un error pero no se
    this->CO2Control();
    this->enable();
    this->forced();
    this->writeIO();
    this->stateIndicator();

    //------Funcion que ejecuta si esta activo el debuger----------
        String strDebugCo2;
        strDebugCo2 = F("-------Console CO2 -------------------------------------\n");
        strDebugCo2 +=F("ValidationPIDCO2.flag : "); 
        strDebugCo2 += String(ValidationPIDCO2.flag,DEC);
        strDebugCo2 = F("\n");
        strDebugCo2 +=F("CO2_Setpoint : "); 
        strDebugCo2 += String(CO2Setpoint,DEC);
        strDebugCo2 = F("\n");
        strDebugCo2 +=F("Sensor CO2 V1 : "); 
        strDebugCo2 += String(analogOutputModule1Values[0],DEC);
        strDebugCo2 = F("\n");
        strDebugCo2 +=F("Sensor CO2 V2 : "; 
        strDebugCo2 += String(analogOutputModule1Values[1],DEC);
        strDebugCo2 = F("\n");
        strDebugCo2 +=F("Salida PID CO2_Control: "); 
        strDebugCo2 += String(CO2PIDOutput,DEC);
        strDebugCo2 = F("\n");
        strDebugCo2 = F("--------------------------------------------\n");
        debugControlCo2(strDebugCo2);
    //------------------------------------------------------------

}

bool controlchamberco2::getAlarmOnGeneral(){
    return alarmOnGeneral;
}

int controlchamberco2::getAnalogOutputModule1ValuesCo2(unsigned char _pos){
  return analogOutputModule1ValuesCo2[_pos];
}


/* funcion para debugear el control de co2 */
void Chamber::debugControlCo2(String mdebug){
  if (debugConsole.co2)
  {
    unsigned long timeConsoleIn = millis();
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
      //--------------aca se imprime todo lo que quiera
      Serial.println(mdebug);
    }   
  }
}


controlchamberco2::~controlchamberco2()
{
}