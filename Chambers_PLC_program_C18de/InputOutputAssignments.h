#ifndef INPUT_OUTPUT_ASSIGNMENTS_INCLUDE
#define INPUT_OUTPUT_ASSIGNMENTS_INCLUDE
// fan 
#define SAFETY_RELAY_RESET      Q0_0
#define INPUT_FAN_1             Q0_2
#define OUTPUT_FAN_1            Q0_3
#define INPUT_FAN_2             Q0_5
#define OUTPUT_FAN_2            Q0_6
#define AEROHEATERS             Q1_0
#define HUMIDITY_WATER_VALVES   Q1_1
//#define HUMIDITY_AIR_VALVES
#define COOLING_REQUEST Q1_2
#define HEATING_REQUEST Q1_7
#define CONTROL_COOLING_REQUEST Q1_4
#define CONTROL_HEATING_REQUEST Q1_5
#define EVAPORATOR_FAN_ACTIVATOR Q1_3
#define ALARM_SET Q0_4


#define DOOR_1_OPEN_DETECT I0_2
#define DOOR_2_OPEN_DETECT 0
#define AUTO_TEL_SELECTOR I0_3
#define EXT_TEMPERATURE_CONTROL I0_4
#define EXT_TEMPERATURE_CONTROL I0_5
#define ETHYLENE_FLOW_IN I0_7 //Entrada analógica sensor de flujo etileno
#define INPUT_FAN_PHASE_1 I0_8
#define OUTPUT_FAN_PHASE_1 I0_9
#define IN_FAN_PHASE_2 I0_10
#define OUT_FAN_PHASE_2 I0_11

#define DEFROST_CYCLE I2_0

#define EXT_COOLER_AVAILABLE I1_0
#define EXT_HEATER_AVAILABLE I1_1
#define COOLER_ACTIVATED I1_2
#define HEATER_ACTIVATED I1_3
#define AIR_PRESSURE_SENSOR I1_4
#define WATER_PRESSURE_SENSOR I1_7

#define GENERAL_SWITCH_DETEC  I1_4  //Detección de interruptor principal
#define MICRO_CUTS_DETECTION  I1_5  //Entrada relé para la detección de microcortes
#define EMERGENCY_STOP        I1_6  //Entrada relé emergencia      



#endif
