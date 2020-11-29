#ifndef  CONST_CONTROL_H
#define CONST_CONTROL_H

////---------------------------------------------------------------------------------
// Macros asosiadas al holdregister para el sistema de control forzado
// fan 
#define FORCED_SAFETY_RELAY_RESET               0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 0)
#define FORCED_INPUT_FAN_1                      0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 1)
#define FORCED_INPUT_FAN_2                      0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 2)
#define FORCED_OUTPUT_FAN_1                     0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 3)
#define FORCED_OUTPUT_FAN_2                     0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 4)
#define FORCED_AEROHEATERS                      0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 5)
#define FORCED_HUMIDITY_WATER_VALVES            0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 6)

//#HUMIDITY_AIR_VALVES
#define FORCED_COOLING_REQUEST                  0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 7)
#define FORCED_HEATING_REQUEST                  0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 8)
#define FORCED_CONTROL_COOLING_REQUEST          0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 9)
#define FORCED_CONTROL_HEATING_REQUEST          0      //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 10)
#define FORCED_EVAPORATOR_FAN_ACTIVATOR         0       //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 11)
#define FORCED_ALARM_SET                        0       //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 12)
#define FORCED_ETHYLENE                         0       //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 336, 13)


// Macros asosiadas al holdregister para verificar si estan habilitados los sistemas de control
#define ENABLE_CONTROL_TEMPERATURE_EXTERNAL       1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 0)
#define ENABLE_CONTROL_TEMPERATURE_AEROTERMO      1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 1)
#define ENABLE_CONTROL_HUMIDITY                   1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 2)
#define ENABLE_CONTROL_CO2                        1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 3)
#define ENABLE_CONROL_C2H4                        1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 4)
#define ENABLE_CONROL_C2H4_FLOW                   1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 5)
#define ENABLE_CONTROL_CO2_ON_OFF                 1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 6)
#define ENABLE_CONTROL_CO2_PID                    1   //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 337, 7)


// Macros asosiadas al holdregister  inicio de ciclo en balance de gaces -  modo mantenimineto
#define INICIO_CICLO_MODO_MANTENIMIENTO             1  //_modbusTCPServer->holdingRegisterReadBit(addressOffset + 338, 7)
#define START2                                      _modbusTCPServer->holdingRegisterReadBit(addressOffset + 0, 2)
#define CLEAR_FINAL_INJECTION_MESSAGE_ACTIVATED     _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 4)
#define CLEAR_FANOUT_ACTIVATED                      _modbusTCPServer->holdingRegisterClearBit(addressOffset + 338, 1)

//Definicion de contantes de las formulas
//Inyecion inicial 
#define REF_C2H4                _modbusTCPServer->holdingRegisterReadFloat(addressOffset + 18)
#define CHAMBER_VOLUMEN         _modbusTCPServer->holdingRegisterRead(addressOffset + 10)

#endif
