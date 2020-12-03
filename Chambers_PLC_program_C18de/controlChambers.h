#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>

//Definiciones de modo del Etileno
#define MODO_INYECCION_INICIAL        1
#define MODO_INYECCION_MANTENIMIENTO  2
#define MODO_DESVERDIZACION           3


class Chamber
{
  private:

  //Estado del modo de desverdozacion-conservasion
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

    

    //funciones privadas para hacer debuggear en cada uno de los sistemas de control
    unsigned char debugEstadoModoDesverdizacion
    void debugControlHumidity();
    void debugControlEthylene();
    void debugControlEthyleneFlow();
    void debugControlCo2();
    void debugControlTemp();
    
    //Auto selector selection
    bool autoSelectorValue = 0;

    



  public:

    Chamber(int chamber,
            ModbusTCPServer* modbusTCPServer,
            ModbusTCPClient* modbusTCPClient1,
            ModbusTCPClient* modbusTCPClient2,
            int &holdingRegisterPerChamber);
    init(int *timerAlarmNoVentilationPointer,int *timerOpenDoorTimeAlarm1Pointer,int *timerOpenDoorTimeAlarm2Pointer);
    writeToEeprom();

    //Control de temperatura
    temperatureControl();
    readTemp();

    //Control PID de Humedad
    humidityControl(int* humidityInyectionTimesPointer,
                    bool* humidityInyectionStatusPointer);
    
    readHumidity();

    //Control PID de ethyleneControl
    ethyleneControl(int* ethyleneInyectionTimesPointer,
                    bool* ethyleneInyectionStatusPointer);
    
    readEthylene();

    //Control PID CO2         
    CO2Control(int* timerInitializationFanPointer);
    readCO2();

    //Control PID de ethyleneFlowRateControl
    ethyleneFlowRateControl();    
    readEthyleneFlowRate();

    //Lectura de las entradas
    readOutputFan1();
    readOutputFan2();

    //mediciones
    getMeasurements();
    getRawValues1();
    //Escritura de los Valores Analogicos
    writeAnalogValues();
    
    //Control de alarmas
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

    void enableInputOutput();



    

    //Zeta de emergencia
//---------> Tarea 4
    void setaEmergency();

    // funcion que atiende y envia la interrupccion del microcut para la asignacion de un clear o un set del h_r 0,3

  //---------> Tarea 5
    void atiendeMicroCutsInterrup(unsigend char *mflagMicroCutsPointer);
//---------> Tarea 5
    void atiendeGeneralSwitchDetect(void);
//---------> Tarea 5
    void setupSafetyRelayReset(void);
//---------> Tarea 11
    void stateAutoTelSelector(void);
    
    ~Chamber(void);



    
    
};
