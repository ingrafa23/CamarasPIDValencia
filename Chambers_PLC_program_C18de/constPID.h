/**
  @file constPID.h
  @brief Archivo que contiene todos los parametros y constantes para normalizar las entradas del PID
  @author ingrafaelangelgg@gmail.com || morenoadriana993@gmail.com
  @date 10/2020

*/
/*************************************************************************************************/
/**
  @brief constantes de normalizacion para la lectura de la Humedad en el control PID

  @param CONST_NORMALIZATION_HUMIDITY_PID valor de la constante para garantizar la lectura del sensor en un rango de 0 a 1
  @param ValueHumidityNormalization valor de la normalizacion entre [0 1] 
  @param valueHumiditySetpointNormalization valor de la normalizacion entre [0 1] 
  @brief constantes del PID para el ETHYLENE_FLOW
  @param KP_ETHYLENE_FLOW_PID valor de la constante KP obtenida del  holdingRegisterRead del protocolo modbus
  @param KI_ETHYLENE_FLOW_PID valor de la constante KI obtenida del  holdingRegisterRead del protocolo modbus
  @param KD_ETHYLENE_FLOW_PID valor de la constante KD obtenida del  holdingRegisterRead del protocolo modbus

  @brief constantes maximas para la salida del PID ETHYLENE CONTROL
  @param ethylenePIDLimitMin valor de la salida minima del PID mA
  @param ETHYLENE__FLOW_PID_LIMIT_MAX valor maximo de la Salida PID mA
  @param ethylenePIDLimitMinClose valor minimo para el cierre de la valvula mA

  @brief constantes del PID para el CO2
  @param PID_CO2_CONTROL_MIN valor minimo
  */
 
  // HUMIDITY PID CONTROL
  #ifndef  CONST_PID_H
  #define CONST_PID_H

  #define CONST_NORMALIZATION_HUMIDITY_PID 100.0
  double valueHumidityNormalization;
  double valueHumiditySetpointNormalization;

  #define OUTPUT_HUMIDITY_LIMITS_MIN 0
  #define OUTPUT_HUMIDITY_LIMITS_MAX 10
  #define SAMPLE_TIME_HUMIDITY 1000  //ms
  #define CONST_DIVISION_KP_HUMIDITY 1000.0
  #define CONST_DIVISION_KI_HUMIDITY 1000.0
  #define CONST_DIVISION_KD_HUMIDITY 1000.0
  
  //ETHYLENE PID CONTROL
  #define CONST_NORMALIZATION_ETHYLENE_PID 1.0
  double valueEthyleneNormalization; 
  double valueEthyleneSetpointNormalization; 
  //#define ETHYLENE_PID_LIMIT_MIN 13500 //minimo PID
  #define ETHYLENE_PID_LIMIT_MIN 13500 //minimo PID
  #define ETHYLENE_PID_LIMIT_MAX 15000//16200 //Maximo PID
  #define ETHYLENE_PID_CLOSE 4000 // minimo forzado
  #define ETHYLENE_PID_OPEN 20000 // maximo forzado 
  
  //518 517
  #define KP_ETHYLENE_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 55)
  #define KI_ETHYLENE_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 56) / 1000.0
  #define KD_ETHYLENE_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 57)


  // ETHYLENE FLOW PID CONTROL
  #define CONST_NORMALIZATION_ETHYLENE_FLOW_PID 1.0
  double valueEthyleneFlowNormalization; 
  double valueEthyleneFlowSetpointNormalization; 

  #define KP_ETHYLENE_FLOW_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 161)
  #define KI_ETHYLENE_FLOW_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 162) / 1000
  #define KD_ETHYLENE_FLOW_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 163)


  
  #define ETHYLENE__FLOW_PID_LIMIT_MIN     13500
  #define ETHYLENE__FLOW_PID_LIMIT_MAX     18000
  #define ETHYLENE__FLOW_PID_LIMIT_CLOSE   4000


  // contantes del PID CO2
  #define KP_CO2_PID    (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 52) / 1.0
  #define KI_CO2_PID    (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 53) / 10.0
  #define KD_CO2_PID    (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 54) / 1.0

  double valueCO2Normalization;  // --> lineas 864, 81
  double valueCO2SetpointNormalization;

  #define PID_CO2_CONTROL_MIN 6500.0
  #define PID_CO2_CONTROL_MAX 20000.0 // 88
  #define CONST_NORMALIZATION_CO2_PID 100.0  //-- 727, 83

  #define CO2_PID_CLOSE 4000 // minimo forzado
  #define CO2_PID_OPEN 20000 // maximo forzado 
 

#endif  
