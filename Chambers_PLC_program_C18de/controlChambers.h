#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "functions.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>

//Definiciones de modo del Etileno
#define MODO_INYECCION_INICIAL 0
#define MODO_INYECCION_MANTENIMIENTO 1


class Chamber
{
  private:
    //-------------------------------------------------
    unsigned char estadoModoDesverdizacion;
    //-------------------------------------------------
    ModbusTCPServer* _modbusTCPServer;
    ModbusTCPClient* _modbusTCPClient1;
    ModbusTCPClient* _modbusTCPClient2;
    PID* ethylenePID;
    PID* humidityPID;
    PID* CO2PID;
    PID* ethyleneFlowPID;
    AnalogFilter<100, 10>* filterEthylFlow;
    AnalogFilter<100, 10>* filterEthyl;
    AnalogFilter<100, 10>* filterCO2;
    AnalogFilter<100, 10>* filterTemp;
    AnalogFilter<100, 10>* filterHum;
    int _chamber;
    int addressOffset;
    int eepromOffset;
    int numHoldingRegistersAddresses = 345;
    int numEepromAddresses = 500;
    int rawValueInputModule1[8];
    double calculatedSensorValues[8];
    int analogOutputModule1Values[4];
    bool gateRaise1 = 0;
    bool gateRaise2 = 0;
    bool doorInyectionAvailableGasesBalance = 0;
    bool ethyleneFlowRateCO2ControlGasesBalance = 0;
    bool initialInyectionWorking = 0;
    bool pidCycleControlTemp = 0;
    bool pidCycleControlHumidity = 0;
    double ethyleneSetpoint, ethylenePIDOutput;
    double humiditySetpoint, humidityPIDOutput;
    double CO2Setpoint, CO2PIDOutput;

    bool previusMode = false;
    bool turnOn = false;

    bool tempUpActivator = false;
    bool tempDownActivator = false;
    bool humidityDownActivator = false;
    bool ethylDownActivator = false;
    bool ethylUpActivator = false;

    

    float inFanPhase;
    float outFanPhase;
    float inFanPhase2;
    float outFanPhase2;

    bool activatorn1 = false;
    bool activatorn2 = false;
    bool activatorn3 = false;
    bool activatorn4 = false;

    float inFanNeutral;
    float outFanNeutral;
    float inFanNeutral2;
    float outFanNeutral2;

    float tempPreviusValue;
    float humidityPreviusValue;
    float ethylPreviusValue;
    float CO2PreviusValue;

    bool alarmSensorTemp1 = false;
    bool alarmSensorHumidity1 = false;
    bool alarmSensorEthyl1 = false;
    bool alarmSensorCO21 = false;

    bool alarmSensorTemp2 = false;
    bool alarmSensorHumidity2 = false;
    bool alarmSensorEthyl2 = false;
    bool alarmSensorCO22 = false;

    int airflowConst = 1;

    double desiredEthyleneFlowRate= 0;
    double actualEthyleneFlowRate= 0;
    double ethyleneFlowPIDOutput= 0;

    bool ethyleneFlowRateControlGasesBalance = 0;
    bool ethyleneFlowRateCO2Control= 0;

    int humidityCycleTOn;
    int humidityCycleTOff;

    float airflowConstant = 0;

    float baseEthyleneFlowRate = 0;

    bool autoTelSelectorBlocker;
    //-------------------------------------------
    //-------------------------------------------
    //banderas de forzado del sistema de control 
    bool flagForcedHumidity = 0;
    bool flagForcedCO2 = 0;
    bool flagForcedEthylene = 0;

    //banderas de enable del sistema de control 
    bool flagEnableControlSystemTemperatureExternal = 0;
    bool flagEnableControlSystemTemperatureAerotermo = 0; 
    bool flagEnableControlSystemHumidity = 0;
    bool flagEnableControlSystemCO2 = 0;
    bool flagEnableControlSystemEthylene = 0;
    bool flagEnableControlSystemEthyleneFlow = 0;
    bool flagEnableControlSystemCO2OnOff = 0;
    bool flagEnableControlSystemCO2Pid = 0;
    

    // -------------------------------------------
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



    //Activador alarma
    bool alarmOn = 0;
 
    //Fornulas del sistema

    double formulaInyecionInicial();



  public:

    Chamber(int chamber,
            ModbusTCPServer* modbusTCPServer,
            ModbusTCPClient* modbusTCPClient1,
            ModbusTCPClient* modbusTCPClient2,
            int &holdingRegisterPerChamber);
    init(int *timerAlarmNoVentilationPointer,int *timerOpenDoorTimeAlarm1Pointer,int *timerOpenDoorTimeAlarm2Pointer);
    writeToEeprom();
    temperatureControl();
    humidityControl(int* humidityInyectionTimesPointer,
                    bool* humidityInyectionStatusPointer);
    ethyleneControl(int* ethyleneInyectionTimesPointer,
                    bool* ethyleneInyectionStatusPointer);
    CO2Control(int* timerInitializationFanPointer);
    ethyleneFlowRateControl();
    readTemp();
    readHumidity();
    readEthylene();
    readCO2();
    readEthyleneFlowRate();
    readOutputFan1();
    readOutputFan2();
    getMeasurements();
    getRawValues1();
    writeAnalogValues();
    alarms(int* timerGoOffAlarmTemperaturePointer, 
           int* timerGoOffAlarmHumidityPointer, 
           int* timerGoOffAlarmEthylenePointer, 
           int* timerGoOffAlarmCO2Pointer,
           int* timerLimitAlarmTemperaturePointer, 
           int* timerLimitAlarmHumidityPointer,  
           int* timerLimitAlarmEthylenePointer, 
           int* timerLimitAlarmCO2Pointer,
           int* timerAlarmNoVentilationPointer,
           int* timerOpenDoorTimeAlarm1Pointer,
           int* timerOpenDoorTimeAlarm2Pointer);

    //Metodo de forzado para las salidas de Humedad, CO2 y Ethylene
    void forcedControl();
    //Metodo para habilitar el sistema de control de humedad, CO2 y etileno
    void enableControl();    
    bool getStateAutoTelSelectorBlocker();
    
    ~Chamber(void);



    
    
};
