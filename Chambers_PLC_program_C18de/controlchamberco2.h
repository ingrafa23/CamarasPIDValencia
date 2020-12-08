#ifndef control_chamber_co2_H
#define control_chamber_co2_H

#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>
#include "consoladebug.h"
#include "mapsensor.h"

//Limites de la Salida PID CO2
  #define PID_CO2_CONTROL_MIN 6500.0
  #define PID_CO2_CONTROL_MAX 20000.0 
  //constante de normalizacion de la entrada del PID CO2
  #define CONST_NORMALIZATION_CO2_PID 100.0  

  // minimo forzado
  #define CO2_PID_CLOSE 4000
   // maximo forzado  
  #define CO2_PID_OPEN 20000 
  

class controlchamberco2
{
private:

    // contantes del PID CO2
    PID* CO2PID;
    double CO2Setpoint, CO2PIDOutput;
    float CO2PreviusValue;

    //Variables de normalizacion
    double valueCO2Normalization;  
    double valueCO2SetpointNormalization;
    double calculatedSensorValues;

    bool alarmOnGeneral;

    //Salidas
    int analogOutputModule1ValuesCo2[2];
    /* data */
    ModbusTCPServer* _modbusTCPServer;
    //---
    mapsensor* _mapsensor;
    //---
    int addressOffset;
    bool alarmSensorCO21 = false;
    bool alarmSensorCO22 = false;
    bool ethyleneFlowRateCO2Control= 0;
    //banderas de forzado del sistema de control 
    bool flagForcedCO2 = 0;

    //banderas de enable del sistema de control 
    bool flagEnableControlSystemCO2OnOff = 0;
    bool flagEnableControlSystemCO2Pid = 0;
    bool flagEnableControlSystemCO2 = 0;

    //funciones privadas para hacer debuggear en cada uno de los sistemas de control
    void debugControlCo2(String mdebug);

    //funciones del etilene flow rate para el balance de gases
    void inyeccionPorVentiladoresCO2(void);

    

    //estrucura de los pines IO
    struct mcontrolChamberCo2IO
    {
        bool inputFan1;
        bool inputFan2;
        bool outputFan1;
        bool outputFan2;
    }controlChamberCo2IO;
      
  
    
    

public:
    controlchamberco2(ModbusTCPServer *modbusTCPServer,int maddressOffset);
    void setup();
    void run(double medidaSendor,bool flagActivadoresVentiladoresBarrido);
    void enable();
    //El barrio y el forzado por Hr se hace en la misma funcion
    void forced(bool flagActivadoresVentiladoresBarrido);
    void alarm();
    void writeIO();
    bool getAlarmOnGeneral();
    //Control PID CO2         
    void CO2Control();
    void readCO2(double medidaSensor);
    int getAnalogOutputModule1ValuesCo2(unsigned char _pos);
    int getMinCo2();
    void stateIndicator(void);
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

    



    ~controlchamberco2();
};

#endif