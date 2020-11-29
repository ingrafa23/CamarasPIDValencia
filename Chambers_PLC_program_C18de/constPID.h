/**
  @file constPID.h
  @brief Archivo que contiene todos los parametros y constantes para normalizar las entradas del PID
  @author ingrafaelangelgg@gmail.com || morenoadriana993@gmail.com
  @date 10/2020

*/
/*************************************************************************************************/

 
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
  
  
  // Constantes ETHYLENE FLOW PID CONTROL
  #define CONST_NORMALIZATION_ETHYLENE_FLOW_PID 1.0
  double valueEthyleneFlowNormalization; 
  double valueEthyleneFlowSetpointNormalization; 


  // Constantes ETHYLENE FLOW PID CONTROL
  #define ETHYLENE__FLOW_PID_LIMIT_MIN     13500
  #define ETHYLENE__FLOW_PID_LIMIT_MAX     18000
  #define ETHYLENE__FLOW_PID_LIMIT_CLOSE   4000

  // contantes del PID CO2
  //Variables de normalizacion
  double valueCO2Normalization;  
  double valueCO2SetpointNormalization;
  
  //Limites de la Salida PID CO2
  #define PID_CO2_CONTROL_MIN 6500.0
  #define PID_CO2_CONTROL_MAX 20000.0 

  //constante de normalizacion de la entrada del PID CO2
  #define CONST_NORMALIZATION_CO2_PID 100.0  

  // minimo forzado
  #define CO2_PID_CLOSE 4000
   // maximo forzado  
  #define CO2_PID_OPEN 20000 
 

#endif  
