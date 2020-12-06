#ifndef CONTROL_CHAMBER_ETHYLENEFLOW_H
#define CONTROL_CHAMBER_ETHYLENEFLOW_H

#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>
#include "consoladebug.h"
#include "mapsensor.h"

//Definiciones de modo del Etileno
#define MODO_INYECCION_INICIAL        1
#define MODO_INYECCION_MANTENIMIENTO  2
#define MODO_DESVERDIZACION           3

// Constantes ETHYLENE FLOW PID CONTROL
#define CONST_NORMALIZATION_ETHYLENE_FLOW_PID 1
// Constantes ETHYLENE FLOW PID CONTROL
#define ETHYLENE__FLOW_PID_LIMIT_MIN     13500
#define ETHYLENE__FLOW_PID_LIMIT_MAX     18000
#define ETHYLENE__FLOW_PID_LIMIT_CLOSE   4000



class controlChamberEthyleneFlow
{
private:
    //Modbus
    ModbusTCPServer* _modbusTCPServer;
    int addressOffset;
    //---
    mapsensor* _mapsensor;
    //---

    PID* ethyleneFlowPID;
    double valueEthyleneFlowNormalization; 
    double valueEthyleneFlowSetpointNormalization;
    double calculatedSensorValues;

    bool doorInyectionAvailableGasesBalance = 0;
    bool ethyleneFlowRateCO2ControlGasesBalance = 0;

    bool ethyleneFlowRateControlGasesBalance = 0;
    bool ethyleneFlowRateCO2Control= 0;

    float baseEthyleneFlowRate = 0;
    bool flagEnableControlSystemEthyleneFlow = 0;

    double desiredEthyleneFlowRate= 0;
    double actualEthyleneFlowRate= 0;
    double ethyleneFlowPIDOutput= 0;
    bool gateRaise1 = 0;
    bool gateRaise2 = 0;

    bool previusMode; //esta se pasa por referencia

    bool turnOn;

    bool alarmOnGeneral;


    struct StructValidationPID  ValidationPIDFlowethylene;
    //Salidas
        
    struct controlChamberEthyleneFlowAnalogOutputModule1
    {
        int value;
        bool flag;
    }analogOutputModule1Values;

    // -------------------------------------------

    //Condiciones de activadores por CO2
    int pidCo2ControlMin;
    double valueCo2_0;
    double valueCo2_1;
    //--------------------------------------
    //funciones del etilene flow rate para el balance de gases
    void modoDesverdizacion( void );
    void modoConservacion( void );
    //modos inyeccion manual
    void inyeccionPorPuerta01 (void);
    void inyeccionPorPuerta02 (void);
    void inyeccionPorVentiladoresCO2(void);
    void inyeccionPorFuga(void);
    bool flagInyeccionPorFuga = 0;
    bool flagIntervaloInyeccionPorFuga = 0;

    //funciones privadas para hacer debuggear en cada uno de los sistemas de control
    //Estado del modo de desverdozacion-conservasion
    //-------------------------------------------------
    unsigned char debugEstadoModoDesverdizacion;
    unsigned char estadoModoDesverdizacion;
    //-------------------------------------------------
    void debugControlEthyleneFlow();



public:
    controlChamberEthyleneFlow(ModbusTCPServer *modbusTCPServer,int maddressOffset);

    void setup();
    void run(double medidaSensor,double mvalueCo2_0,double mvalueCo2_1,int mpidCo2ControlMin);
    void enable();
    void forced();
    void alarm();
    
    
    //Control PID de ethyleneControl
    void ethyleneFlowControl();
    
    void readEthyleneFlow(double medidaSensor);
    int getAnalogOutputModule1ValuesEthyleneFlow();
    bool getAnalogOutputModule1FlagEthyleneFlow();
    void stateIndicator(void);


    ~controlChamberEthyleneFlow();
};




#endif