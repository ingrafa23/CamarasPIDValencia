#ifndef    VARIABLE_ALARM_INCLUDE
#define VARIABLE_ALARM_INCLUDE


extern double hour2seg(double mT); /// variables en segundo que equivale a 24Horas

#define MAX_TIME_ALARM_NO_VENTILATION    _modbusTCPServer->holdingRegisterRead(addressOffset + 71) //hour2seg(24.0)


#define VALUE_ACTUAL_INPUT_FAN1      digitalRead(INPUT_FAN_1)
#define VALUE_ACTUAL_OUTPUT_FAN1     digitalRead(OUTPUT_FAN_1)
#define VALUE_ACTUAL_INPUT_FAN2      digitalRead(INPUT_FAN_2)
#define VALUE_ACTUAL_OUTPUT_FAN2     digitalRead(OUTPUT_FAN_2)

extern bool flagTimerAlarmNoVentilationPointer;

// variables alarm open door 01
#define VALUE_DOOR_1                 digitalRead(DOOR_1_OPEN_DETECT)
#define MAX_TIME_OPEN_DOOR_1         _modbusTCPServer->holdingRegisterRead(addressOffset + 72)
extern bool flagTimerOpenDoorTimeAlarm1Pointer;

// variables alarm open door 01
#define VALUE_DOOR_2                 digitalRead(DOOR_2_OPEN_DETECT)
#define MAX_TIME_OPEN_DOOR_2         _modbusTCPServer->holdingRegisterRead(addressOffset + 73)
extern bool flagTimerOpenDoorTimeAlarm2Pointer;




#endif
