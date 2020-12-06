#include "controlChamberEthyleneFlow.h"
#include "HoldingRegisterControl.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "InputOutputAssignments.h"
#include "miscellaneous.h"
#include "consoladebug.h"
#include "mapsensor.h"


controlChamberEthyleneFlow::controlChamberEthyleneFlow(ModbusTCPServer *modbusTCPServer,int maddressOffset){
    _modbusTCPServer = modbusTCPServer;
    addressOffset = maddressOffset;

    //-----------------
    int mConts =CONST_NORMALIZATION_ETHYLENE_FLOW_PID;
    _mapsensor = new mapsensor(_modbusTCPServer,
                        addressOffset + 274,             // C2h4 flow Measure
                        addressOffset + 115,             // LowLimit1
                        addressOffset + 116,             // HighLimit1
                        addressOffset + 117,             // zeroSensor1
                        addressOffset + 118,             // spanSensor1
                        mConts);   // constante de normalización co2
    //-----------------
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

}

void controlChamberEthyleneFlow::readEthyleneFlow(double medidaSensor){
  _mapsensor->mapFloatMeasurementSensor(medidaSensor);
  valueEthyleneFlowNormalization = _mapsensor->getValueSensorNormaliced();
  calculatedSensorValues = _mapsensor->getValueSensor();
}

void controlChamberEthyleneFlow::ethyleneFlowControl(){
    /*BEGIN CONDITION ENABALE CONTROL SYSTEM ETHYLENE*/
    // Entrada modo trabajo desverdizacion
    if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 0))
    {
        if (_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 1))     
        {
            //Entrada en modo Conservacion
            modoConservacion();
        }
        else 
        {
            //Entrada en modo Desverdizacion
            modoDesverdizacion();
        }
    }
    else
    {
        ethyleneFlowPID->SetMode(MANUAL);

        analogOutputModule1Values.value = ETHYLENE__FLOW_PID_LIMIT_CLOSE;
        analogOutputModule1Values.flag = 1;

        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
    }
}

void controlChamberEthyleneFlow::alarm(){
    
    
}

void controlChamberEthyleneFlow::enable(){
    if (!ENABLE_ETHYLENE)
    {
        analogOutputModule1Values.value = ETHYLENE__FLOW_PID_LIMIT_CLOSE;
        analogOutputModule1Values.flag = 1;
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
    }
    
}

void controlChamberEthyleneFlow::forced(){
    if (FORCED_ETHYLENE)
    {
        analogOutputModule1Values.value = ETHYLENE__FLOW_PID_LIMIT_MAX;
        analogOutputModule1Values.flag = 1;
    }
    
}

void controlChamberEthyleneFlow::stateIndicator(){
    if (analogOutputModule1Values.value > ETHYLENE__FLOW_PID_LIMIT_CLOSE)
    {
        _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
    }
    else
    {
        _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
    }
}

/**
 * @brief run es la funcion principal que ejecuta todo el sistema de control de ethylkeno flow
 * @param medidaSensor es el valos de la medida del sensor
 * @param mvalueCo2_0 es el valor obtenido en el sistema co2 posicion de la carta 0
 * @param mvalueCo2_1 es el valor obtenido en el sistema co2 posicion de la carta 1
 * @param mpidCo2ControlMin es el valor minimo permitido en el sistema de control co2
*/
void controlChamberEthyleneFlow::run(double medidaSensor,double mvalueCo2_0,double mvalueCo2_1,int mpidCo2ControlMin){
    valueCo2_0 = mvalueCo2_0;
    valueCo2_1 = mvalueCo2_1;
    pidCo2ControlMin = mpidCo2ControlMin;

    this->readEthyleneFlow(medidaSensor);
    //this->alarm();
    this->ethyleneFlowControl();
    this->enable();
    this->forced();
    this->stateIndicator();



}


int controlChamberEthyleneFlow::getAnalogOutputModule1ValuesEthyleneFlow(){
    return analogOutputModule1Values.value;
}

bool controlChamberEthyleneFlow::getAnalogOutputModule1FlagEthyleneFlow(){
    bool m_resp = analogOutputModule1Values.flag;
    analogOutputModule1Values.flag = 0; // indicar que fue atendida
    return m_resp;
}

///Funciones privadas


///------------------------------------------------------------------------
/**
 * @brief modoDesverdizacion 
*/
void controlChamberEthyleneFlow::modoDesverdizacion()
{
  
  if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4)) //si balance de gases activado //SI las alarmas saltan entra en balance de gases
      {
        //
        //Serial.print("Hay modificaciones del PID:"); Serial.println(_modbusTCPServer->holdingRegisterReadBit(addressOffset + 250, 4));
        //Hay modificaciones del PID        
        if(_modbusTCPServer->holdingRegisterReadBit(addressOffset + 250, 4) && _modbusTCPServer->holdingRegisterReadBit(addressOffset + 338, 7))
        { //inyecion inicial inicializacion

          //Pone activador inyección inicial etileno a 0
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 250, 4);
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 7);
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 10);

                   
          // Inicio de conservacion de ciclo activado
          //Paramos inyeccion por mantenimiento
          _modbusTCPServer->holdingRegisterClearBit(addressOffset + 0, 2); 

          //Este es el timer de inyección le cargamos el valor de inyección inicial
          
          setTimerIntEthyleneFlow(_modbusTCPServer->holdingRegisterRead(addressOffset + 59));
          //Serial.print("desiredEthyleneFlowRate :"); Serial.println(String(desiredEthyleneFlowRate, 4));
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
                
        ethyleneFlowPID->SetMode(AUTOMATIC);
        ethyleneFlowPID->SetTunings(KP_ETHYLENE_FLOW_PID,KI_ETHYLENE_FLOW_PID,KD_ETHYLENE_FLOW_PID);
        ethyleneFlowPID->Compute();

        analogOutputModule1Values.value = ethyleneFlowPIDOutput;
        analogOutputModule1Values.flag = 1;

        ValidationPIDFlowethylene.valor = ethyleneFlowPIDOutput;

        ValidationPID(&ValidationPIDFlowethylene,ETHYLENE__FLOW_PID_LIMIT_MIN);
        if (ValidationPIDFlowethylene.flag)
        {
          analogOutputModule1Values.value = ETHYLENE__FLOW_PID_LIMIT_CLOSE;
          analogOutputModule1Values.flag = 1;
        }

      }
}


/**
 * @brief Inyeccion por puerta 01
*/
void controlChamberEthyleneFlow::inyeccionPorPuerta01(){
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
void controlChamberEthyleneFlow::inyeccionPorPuerta02(){
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
void controlChamberEthyleneFlow::inyeccionPorVentiladoresCO2(){
    // C2H4 control selection && ethyleneFlowRateCO2ControlGasesBalance
      if (!_modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 4))
      {
        //-----
        if (valueCo2_0 > pidCo2ControlMin) //Iniciamos la inyección de etileno si
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
          double airflow = ((double)valueCo2_0 +
                          (double)valueCo2_1) *
                          (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 136);

          //Calculo para la reglas de las mezclas
          double ratioConcentration = _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18) /
                                    _modbusTCPServer->holdingRegisterReadLong(addressOffset + 12);

          double extractionEthyleneFlowRate = ((airflow * ratioConcentration) / (1 - ratioConcentration));
          desiredEthyleneFlowRate += extractionEthyleneFlowRate; //lo añado al setpoint
          _modbusTCPServer->holdingRegisterSetBit(addressOffset + 338, 1);
          
          //
          Serial.print("inyeccion por ventiladores co2 :"); Serial.println(valueCo2_0 > pidCo2ControlMin);
          Serial.print("extractionEthyleneFlowRate :"); Serial.println(extractionEthyleneFlowRate);
          Serial.print("desiredEthyleneFlowRate :"); Serial.println(String(desiredEthyleneFlowRate, 4));
        }
        
      }
}

/**
 * @brief Inyeccion por fugas y perdidas
*/
void controlChamberEthyleneFlow::inyeccionPorFuga(){
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
void controlChamberEthyleneFlow::modoConservacion()
{

      ethyleneFlowPID->SetMode(MANUAL);
      
      analogOutputModule1Values.value = ETHYLENE__FLOW_PID_LIMIT_CLOSE;
      analogOutputModule1Values.flag = 1;

      _modbusTCPServer->holdingRegisterSetBit(addressOffset + 250, 4);
      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1);
}

//----------------------------------------

/* funcion para debugear el control del flujo de etileno */
void controlChamberEthyleneFlow::debugControlEthyleneFlow(){
  if (debugConsole.ethyleneFlow)
  {
    unsigned long timeConsoleIn = abs(debugLastTime.ethyleneFlow -  millis());
    if(timeConsoleIn>1000){//para que se imprima cada 1000ms
      debugLastTime.ethyleneFlow = millis();
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
      Serial.print("Lectura Sensor Ethylene Flow  :"); Serial.println(calculatedSensorValues);
      Serial.print("PID ethyleneFlowPIDOutput : ");Serial.println(ethyleneFlowPIDOutput);
      Serial.println("-------________________------------------------------------------");
    }    
  }
}





controlChamberEthyleneFlow::~controlChamberEthyleneFlow()
{
    
}