#ifndef  HOLDING_REGISTER_CONTROL_H 
#define HOLDING_REGISTER_CONTROL_H

////---------------------------------------------------------------------------------

//Macros asociadas al los holdregister del control PID

//Constantes PID ETHYLENE CONTROL
  #define KP_ETHYLENE_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 55)
  #define KI_ETHYLENE_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 56) / 1000.0
  #define KD_ETHYLENE_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 57)

// Constantes ETHYLENE FLOW PID CONTROL
  #define KP_ETHYLENE_FLOW_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 161)
  #define KI_ETHYLENE_FLOW_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 162) / 1000
  #define KD_ETHYLENE_FLOW_PID   _modbusTCPServer->holdingRegisterRead(addressOffset + 163)

// contantes del PID CO2
  #define KP_CO2_PID    (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 52) / 1.0
  #define KI_CO2_PID    (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 53) / 10.0
  #define KD_CO2_PID    (double)_modbusTCPServer->holdingRegisterRead(addressOffset + 54) / 1.0
  

// Macros asosiadas al holdregister para el sistema de control forzado
// fan 
#define FORCED_SAFETY_RELAY_RESET               _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 0)
#define FORCED_INPUT_FAN_1                      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 1)
#define FORCED_INPUT_FAN_2                      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 2)
#define FORCED_OUTPUT_FAN_1                     _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 3)
#define FORCED_OUTPUT_FAN_2                     _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 4)
#define FORCED_AEROHEATERS                      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 5)
#define FORCED_HUMIDITY_WATER_VALVES            _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 6)

#define FORCED_HUMIDITY_AIR_VALVES              _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 14)
#define FORCED_COOLING_REQUEST                  _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 7)
#define FORCED_HEATING_REQUEST                  _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 8)
#define FORCED_CONTROL_COOLING_REQUEST          _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 9)
#define FORCED_CONTROL_HEATING_REQUEST          _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 10)
#define FORCED_EVAPORATOR_FAN_ACTIVATOR         _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 11)
#define FORCED_ALARM_SET                        _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 12)
#define FORCED_ETHYLENE                         _modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 13)

#define FORCED_AUTOTEL_SELECTOR_HR              _modbusTCPServer->holdingRegisterReadBit(addressOffset + 256, 10)

// Macros asociadas al holdregister  inicio de ciclo en balance de gaces -  modo mantenimineto
#define INICIO_CICLO_MODO_MANTENIMIENTO             1  //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 338, 7)
#define START2                                      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 2)
#define CLEAR_FINAL_INJECTION_MESSAGE_ACTIVATED     _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 4)
#define CLEAR_FANOUT_ACTIVATED                      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1)

// Macros asociadas al holdregister para verificar si las entradas o salidas estan HABILITADAS 
#define ENABLE_SAFETY_RELAY_RESET           _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 13)
#define ENABLE_INPUT_FAN_1                  _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 5)
#define ENABLE_OUTPUT_FAN_1                 _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 6)
#define ENABLE_INPUT_FAN_2                  _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 7)
#define ENABLE_OUTPUT_FAN_2                 _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 8)
#define ENABLE_AEROHEATERS                  _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 2)
#define ENABLE_HUMIDITY_WATER_VALVES        _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 3)
#define ENABLE_HUMIDITY_AIR_VALVES          _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 14)
#define ENABLE_COOLING_REQUEST              _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 9)
#define ENABLE_HEATING_REQUEST              _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 10)  
#define ENABLE_CONTROL_COOLING_REQUEST      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 11)
#define ENABLE_CONTROL_HEATING_REQUEST      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 12)
#define ENABLE_EVAPORATOR_FAN_ACTIVATOR     _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 1)
#define ENABLE_ALARM_SET                    _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 0)
#define ENABLE_ETHYLENE                     _modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 4)

#define ENABLE_AUTOTEL_SELECTOR_HR          _modbusTCPServer->holdingRegisterReadBit(addressOffset + 256, 10)
#define SET_AUTOTEL_SELECTOR_HR             _modbusTCPServer->holdingRegisterSetBit(addressOffset + 256, 10)
#define CLEAR_AUTOTEL_SELECTOR_HR           _modbusTCPServer->holdingRegisterClearBit(addressOffset + 256, 10)

struct strHoldingRegisterControlEnable
{
  unsigned char enable_safety_relay_reset;           
  unsigned char enable_input_fan_1;                  
  unsigned char enable_output_fan_1;                
  unsigned char enable_input_fan_2;                
  unsigned char enable_output_fan_2;               
  unsigned char enable_aeroheaters;              
  unsigned char enable_humidity_water_valves;      
  unsigned char enable_humidity_air_valves;    
  unsigned char enable_cooling_request;    
  unsigned char enable_heating_request;    
  unsigned char enable_control_cooling_request; 
  unsigned char enable_control_heating_request; 
  unsigned char enable_evaporator_fan_activator;
  unsigned char enable_ethylene;
  unsigned char enable_autotel_selector_hr;
};

extern struct strHoldingRegisterControlEnable holdingRegisterControlEnable;

//-----------------------------------------------------------------------


//#define EMERGENCY_STOP I0_0
#define DOOR_1_OPEN_DETECT I0_2
#define DOOR_2_OPEN_DETECT 0




//Definicion de contantes de las formulas
//Inyecion inicial 
#define REF_C2H4                _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18)
#define CHAMBER_VOLUMEN         _modbusTCPServer->holdingRegisterRead(addressOffset + 10)

// definicion del auto selector
#define AUTO_TEL_SELECTOR_STATE    _modbusTCPServer->holdingRegisterReadBit(addressOffset + 256, 11)



#endif
