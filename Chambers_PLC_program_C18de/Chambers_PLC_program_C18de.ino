#include <Ethernet.h>
#include <ArduinoModbus.h>
#include "miscellaneous.h"
#include "controlChambers.h"
#include "variablesAlarm.h"
#include "variablestimer.h"
#include "consoladebug.h"
#include "InputOutputAssignments.h"



// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xF0};
IPAddress ownIp(192, 168, 1, 6);
IPAddress analogInputModule1(192, 168, 1, 10);
IPAddress analogOutputModule1(192, 168, 1, 11);
IPAddress dns(192, 168, 1, 1);



// Server instances creation
EthernetServer server(502);
ModbusTCPServer modbusTCPServer;

//Client instances creation
EthernetClient ethernetClient1;
EthernetClient ethernetClient2;
ModbusTCPClient analogInputModule1Client(ethernetClient1);
ModbusTCPClient analogOutputModule1Client(ethernetClient2);

int holdingRegisterAmount = 0;

int ethyleneInyectionTimes[2] = {0, 0};
int* ethyleneInyectionTimesPointer = &ethyleneInyectionTimes[0];

bool ethyleneInyectionStatus = 0;
bool* ethyleneInyectionStatusPointer = &ethyleneInyectionStatus;

/*
int humidityInyectionTimes[2] = {0, 0};
int* humidityInyectionTimesPointer = &humidityInyectionTimes[0];

bool humidityInyectionStatus = 0;
bool *humidityInyectionStatusPointer = &humidityInyectionStatus;
*/
int timerNeutro1 = 0;
int* timerNeutro1Pointer = &timerNeutro1;

int timerNeutro2 = 0;
int* timerNeutro2Pointer = &timerNeutro2;

int timerNeutro3 = 0;
int* timerNeutro3Pointer = &timerNeutro3;

int timerNeutro4 = 0;
int* timerNeutro4Pointer = &timerNeutro4;

int timerGoOffAlarmTemperature = 0;
int* timerGoOffAlarmTemperaturePointer = &timerGoOffAlarmTemperature;

/*
int timerGoOffAlarmEthylene = 0;
int* timerGoOffAlarmEthylenePointer = &timerGoOffAlarmEthylene;
*/


int timerLimitAlarmTemperature = 0;
int* timerLimitAlarmTemperaturePointer = &timerLimitAlarmTemperature;


/*
int timerLimitAlarmEthylene = 0;
int* timerLimitAlarmEthylenePointer = &timerLimitAlarmEthylene;
*/



int timerInitializationFan = 0;
int* timerInitializationFanPointer = &timerInitializationFan;

int timerAlarmNoVentilation;
int *timerAlarmNoVentilationPointer = &timerAlarmNoVentilation;

int timerOpenDoorTimeAlarm1;
int *timerOpenDoorTimeAlarm1Pointer = &timerOpenDoorTimeAlarm1;

int timerOpenDoorTimeAlarm2;
int *timerOpenDoorTimeAlarm2Pointer = &timerOpenDoorTimeAlarm2;


int delayConnection;

//Variable y funcion de micro corte
//------>Tarea 5
unsigned char flagAttacMicroCuts = 0;
unsigned char *flagAttacMicroCutsPointer = &flagAttacMicroCuts;
void attacMicroCuts();
//-------------------------------

Chamber chamber1(0,
                 &modbusTCPServer,
                 &analogInputModule1Client,
                 &analogOutputModule1Client,
                 holdingRegisterAmount);


void setup() {
  Serial.begin(57600);
  while (!Serial);

  Ethernet.begin(mac, ownIp, dns);
  server.begin();

  // Start the Modbus TCP server
  if (!modbusTCPServer.begin(1)) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }

  // Start the Modbus TCP client Analog Input Module 1
  if (!analogInputModule1Client.begin(analogInputModule1, 502)) {
    Serial.println("Analog Input Module 1 failed to connect!");
  } else {
    Serial.println("Analog Input Module 1 connected");
  }

  // Start the Modbus TCP client Analog Output Module 1
  if (!analogOutputModule1Client.begin(analogOutputModule1, 502)) {
    Serial.println("Analog Output Module 1 failed to connect!");
  } else {
    Serial.println("Analog Output Module 1 connected");
  }

  Serial.print("Número de holding registers: ");
  Serial.println(holdingRegisterAmount);


  // configure holding registers
  modbusTCPServer.configureHoldingRegisters(0, holdingRegisterAmount);


  configureTimer5();

  //------>Tarea 5
  //------->  Interrupcion por cambio de estado de pines
  attachInterrupt(digitalPinToInterrupt(MICRO_CUTS_DETECTION), attacMicroCuts, CHANGE);

  
  chamber1.init(timerAlarmNoVentilationPointer,
                  timerOpenDoorTimeAlarm1Pointer,
                  timerOpenDoorTimeAlarm2Pointer);
  
  //---------> Tarea 5
  chamber1.setupSafetyRelayReset();

}

unsigned int numeroPasos = 0;

void loop() {

  //Serial.print("Timer etileno : "); Serial.println(*(ethyleneInyectionTimesPointer + *ethyleneInyectionStatusPointer));
  
  //------>Tarea 5
  //Atiende la interrupcion de MicroCuts
  chamber1.atiendeMicroCutsInterrup(&flagAttacMicroCutsPointer); 
  //----------------------
  //---------> Tarea 5
  chamber1.atiendeGeneralSwitchDetect();
  //---------------------------

  EthernetClient client = server.available();

  if (client)
  {
    // let the Modbus TCP accept the connection
    modbusTCPServer.accept(client);
    //Serial.println("Client accepted");

    if (client.connected())
    {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();

      // aqui debe estar el codigo principa, por que hay muchas cosas que usa modbus
      // entonces pierde la coneccion o no hay cliente y se sigue metiendo
    }
  }
  /*
  else
  {
    //client.stop();
    //Serial.println("Client stoped");
    //Serial.println("perdio conecion o no hay cliente");
  }
  */
  


  

  if (!analogInputModule1Client.connected()) {
    // client not connected, start the Modbus TCP client
    Serial.println("Attempting to connect to Modbus TCP server");

    if (!analogInputModule1Client.begin(analogInputModule1, 502)) {
      Serial.println("Analog Input Module 1 failed to connect!");
    } else {
      Serial.println("Analog Input Module 1 connected");
    }
  } else
  {
    chamber1.getRawValues1();
  }
  


  chamber1.getMeasurements();

 
  chamber1.temperatureControl();

  chamber1.humidityControl(humidityInyectionTimesPointer,
                              humidityInyectionStatusPointer);
                              
  
  chamber1.ethyleneControl(ethyleneInyectionTimesPointer,
                           ethyleneInyectionStatusPointer);
  chamber1.CO2Control(timerInitializationFanPointer);
  chamber1.ethyleneFlowRateControl();
  chamber1.alarms(timerGoOffAlarmTemperaturePointer,
                  timerGoOffAlarmHumidityPointer,
                  timerGoOffAlarmEthylenePointer,
                  timerGoOffAlarmCO2,
                  timerLimitAlarmTemperaturePointer,
                  timerLimitAlarmHumidityPointer,
                  timerLimitAlarmEthylenePointer,
                  timerLimitAlarmCO2Pointer,
                  timerAlarmNoVentilationPointer,
                  timerOpenDoorTimeAlarm1Pointer,
                  timerOpenDoorTimeAlarm2Pointer);
 
  

  


//Serial.print("numeroPasos"); Serial.println(numeroPasos++);



  if (modbusTCPServer.holdingRegisterRead(260))
  {
    chamber1.writeToEeprom();
  }
  


  if (!analogOutputModule1Client.connected()) {
    // client not connected, start the Modbus TCP client
    Serial.println("Attempting to connect to Modbus TCP server");

    if (!analogOutputModule1Client.begin(analogOutputModule1, 502)) {
      Serial.println("Analog Output Module 1 failed to connect!");
    } else {
      Serial.println("Analog Output Module 1 connected");
    }
  } else
  {
    //-----> Tarea 2
    //enable control
    chamber1.enableControl();
    //enable Input Output
    chamber1.enableInputOutput();
    //forced control
    //-----> Tarea 3
    chamber1.forcedControl();
    
    chamber1.writeAnalogValues();
    //Zeta de Emergencia
    //-----> Tarea 4
    chamber1.setaEmergency();


  }
  //Funcion que ejecuta los comandos recibido del interprete o cosola debug
  tareaMainInterprete();
  //Ejecuta el interprete de las acciones de los comados
  interpreteEjecuta()
}

//Interrupcion del pin MICRO_CUTS_DETECTION
//--->Tarea 5
void attacMicroCuts()
{
  flagAttacMicroCuts = 1;
}

ISR(TIMER5_OVF_vect)
{
  if (*(ethyleneInyectionTimesPointer + *ethyleneInyectionStatusPointer) > 0)
  {
    *(ethyleneInyectionTimesPointer + *ethyleneInyectionStatusPointer) -= 1;
    
  }

  if (*(humidityInyectionTimesPointer + *humidityInyectionStatusPointer) > 0)
  {
    *(humidityInyectionTimesPointer + *humidityInyectionStatusPointer) -= 1;
  }

  if (*timerNeutro1Pointer > 0)
  {
    *timerNeutro1Pointer -= 1;
  }

  if (*timerNeutro2Pointer > 0)
  {
    *timerNeutro2Pointer -= 1;
  }

  if (*timerNeutro3Pointer > 0)
  {
    *timerNeutro3Pointer -= 1;
  }

  if (*timerNeutro4Pointer > 0)
  {
    *timerNeutro4Pointer -= 1;
  }

  if (*timerGoOffAlarmTemperaturePointer > 0)
  {
    *timerGoOffAlarmTemperaturePointer -= 1;
  }

  if (timerGoOffAlarmhumidity > 0)
  {
    timerGoOffAlarmhumidity -= 1;
  }

  if (timerGoOffAlarmEthylene > 0)
  {
    timerGoOffAlarmEthylene -= 1;
  }

  if (timerGoOffAlarmCO2 > 0)
  {
    timerGoOffAlarmCO2 -= 1;
  }

  if (*timerLimitAlarmTemperaturePointer > 0)
  {
    *timerLimitAlarmTemperaturePointer -= 1;
  }

  if (timerLimitAlarmHumidity > 0)
  {
    timerLimitAlarmHumidity -= 1;
  }

  if (timerLimitAlarmEthylene > 0)
  {
    timerLimitAlarmEthylene -= 1;
  }

  if (timerLimitAlarmCO2 > 0)
  {
    timerLimitAlarmCO2 -= 1;
  }

  if (*timerInitializationFanPointer > 0)
  {
    *timerInitializationFanPointer -= 1;
  }

  if(*timerAlarmNoVentilationPointer > 0){
    *timerAlarmNoVentilationPointer -= 1;
  } else{
    *timerAlarmNoVentilationPointer = 0;
  }

  if(*timerOpenDoorTimeAlarm1Pointer > 0){
    *timerOpenDoorTimeAlarm1Pointer -= 1;
  } else{
    *timerOpenDoorTimeAlarm1Pointer = 0;
  }

  if(*timerOpenDoorTimeAlarm2Pointer > 0){
    *timerOpenDoorTimeAlarm2Pointer -= 1;
  } else{
    *timerOpenDoorTimeAlarm2Pointer = 0;
  }


  //------------------------------------------
  // temporizacion Inyeccion Inicial 20 min
  if(getTimerIntEthyleneFlow()>0){

    setTimerIntEthyleneFlow(getTimerIntEthyleneFlow() - 1);

  }else{
    setTimerIntEthyleneFlow(0);
  }

  // temporizacion Inyeccion Por puerta 01 10seg 
  if(getTimerOnDoor01EthyleneFlow()>0){

    setTimerOnDoor01EthyleneFlow(getTimerOnDoor01EthyleneFlow() - 1);

  }else{
    setTimerOnDoor01EthyleneFlow(0);
  }

  // temporizacion Inyeccion Por puerta 02 10seg 
  if(getTimerOnDoor02EthyleneFlow()>0){

    setTimerOnDoor02EthyleneFlow(getTimerOnDoor02EthyleneFlow() - 1);

  }else{
    setTimerOnDoor02EthyleneFlow(0);
  }

  //
  if(getTimerOnFugaEthyleneFlow()>0){
    setTimerOnFugaEthyleneFlow(getTimerOnFugaEthyleneFlow() - 1);
  }else
  {
    setTimerOnFugaEthyleneFlow(0);
  }
  
  //
  if(getTimerOnFugaEthyleneFlow()==0){

    if(getTimerInyeccionFugaEthyleneFlow()>0){
      setTimerInyeccionFugaEthyleneFlow(getTimerInyeccionFugaEthyleneFlow()-1);
    }
    else
    {
      setTimerInyeccionFugaEthyleneFlow(0);
    }

  }

  //variables del timer atiende a general switch detect

  if (timerMicroCut.timer10 > 0)
  {
    timerMicroCut.timer10--;
  }

  if (timerMicroCut.timer03 > 0)
  {
    timerMicroCut.timer03--;
  }

  //-----------------------------------------------
  

  TCNT5H = 0xC2;
  TCNT5L = 0xF7;
}



