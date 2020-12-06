#include <ArduinoModbus.h>
#include <Arduino.h>
#include <EEPROM.h>
#include "miscellaneous.h"
#include "InputOutputAssignments.h"
#include <PID_v1.h>
#include <Filter.h>

//incluimos las nuevas clases de los sistemas de control
#include "controlchamberco2.h"
#include "controlChamberEthylene.h"
#include "controlchambershumidity.h"
#include "controlChamberEthyleneFlow.h"
#include "controlChamberTemperature.h"
//lectura de sensores
#include "readsensor.h"
#include "mapsensor.h"



//Pociones del vector de los calculos
#define SensorOuputTemperatureValuePos            0
#define SensorOuputHumidityValuePos               1
#define SensorOuputEthyleneValuePos               2
#define SensorOuputCo2ValuePos                    3
#define SensorOuputEthyleneFlowValuePos           4
#define SensorOutputFan1ValuePos                  5
#define SensorOutputFan2ValuePos                  6


//Entrada de los sensores
#define NumberRawValueInputModule1        4

#define rawValueInputModule1Co2           0
#define rawValueInputModule1Ethylene      1
#define rawValueInputModule1Temp          2
#define rawValueInputModule1Hum           3

//Posiciones de las Salidas de las cartas Analogicas
#define NumberanalogOutputModule1Values       4
#define analogOutputModule1ValuesCo2_0        0
#define analogOutputModule1ValuesCo2_1        1
#define analogOutputModule1ValuesEthylene     2
#define analogOutputModule1ValuesTemp         3



class Chamber
{
  private:

  readsensor * readsensorInput;
  controlchamberco2 * _controlchamberco2;
  controlchambershumidity* _controlchambershumidity;
  controlChamberEthylene * _controlChamberEthylene;
  controlChamberEthyleneFlow* _controlChamberEthyleneFlow;
  controlChamberTemperature* _controlChamberTemperature;
    
    ModbusTCPServer* _modbusTCPServer;
    ModbusTCPClient* _modbusTCPClient1;
    ModbusTCPClient* _modbusTCPClient2;
   
    int _chamber;
    int addressOffset;
    int eepromOffset;
    int numHoldingRegistersAddresses = 370;
    int numEepromAddresses = 500;
      
    int analogOutputModule1Values[4];
    
    //-------------------------------------------
    //---
    mapsensor* _mapsensorReadOutputFan1;
    mapsensor* _mapsensorReadOutputFan2;
    //---
    double calculatedSensorValuesOutputFan1;
    double calculatedSensorValuesOutputFan2;
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
    
    //Activador alarma
    bool alarmOn = 0;
    
    //Auto selector selection
    bool autoSelectorValue;

    struct mcontrolChambersIO
    {
      bool alarmSet;
      bool evaporatorFanActivator;
      bool safetyRelayReset;
    }controlChambersIO;

 
  public:

    Chamber(int chamber,
            ModbusTCPServer* modbusTCPServer,
            ModbusTCPClient* modbusTCPClient1,
            ModbusTCPClient* modbusTCPClient2,
            int &holdingRegisterPerChamber);
    void init();

    void writeToEeprom();

    //Lectura de las entradas
    void readOutputFan1(double medidaSensor);
    void readOutputFan2(double medidaSensor);

    //mediciones
    void measurements();
    
    //Escritura de los Valores Analogicos
    void writeAnalogValues();
    
    //Control de alarmas
    void alarms();

    //Escritura de los Valores Analogicos
    void run();

    void readAnalogValuesInput();

    //Metodo de forzado para las salidas de Humedad, CO2 y Ethylene
    void forced();
    //Metodo para habilitar el sistema de control de humedad, CO2 y etileno
    void enable(); 

    //asignar los valores a las salidas segun la operacion del sistema
    void writeIODigital();
    
    //Zeta de emergencia
    //---------> Tarea 4
    void setaEmergency();

    // funcion que atiende y envia la interrupccion del microcut para la asignacion de un clear o un set del h_r 0,3

    //---------> Tarea 5
    void atiendeMicroCutsInterrup(unsigned char *mflagMicroCutsPointer);
    //---------> Tarea 5
    void atiendeGeneralSwitchDetect();
    //---------> Tarea 5
    void setupSafetyRelayReset();
    //---------> Tarea 11
    void stateAutoTelSelector();

    //----------> Tarea 17
    // indicadoresEstados es la funcion encargada de verficar las Salidas digitales 
    void stateIndicator();
    
    ~Chamber();

    
    
    
};
