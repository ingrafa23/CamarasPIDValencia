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


int timerNeutro1 = 0;
int* timerNeutro1Pointer = &timerNeutro1;

int timerNeutro2 = 0;
int* timerNeutro2Pointer = &timerNeutro2;

int timerNeutro3 = 0;
int* timerNeutro3Pointer = &timerNeutro3;

int timerNeutro4 = 0;
int* timerNeutro4Pointer = &timerNeutro4;





int timerInitializationFan = 0;
int* timerInitializationFanPointer = &timerInitializationFan;


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

  chamber1.init();
  //---------> Tarea 5
  chamber1.setupSafetyRelayReset();

}

unsigned int numeroPasos = 0;

void loop() {
 
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
    }
  }


  if (modbusTCPServer.holdingRegisterRead(260))
  {
    chamber1.writeToEeprom();
  }
  
  
  if (!analogInputModule1Client.connected() || !analogOutputModule1Client.connected()) {
    
    Serial.println("Attempting to connect to Modbus TCP server");

    if (!analogOutputModule1Client.begin(analogOutputModule1, 502)) {
      Serial.println("Analog Output Module 1 failed to connect!");
    } else {
      Serial.println("Analog Output Module 1 connected");
    }

    //////-------------------------------------------------------------
    Serial.println("Attempting to connect to Modbus TCP server");

    if (!analogInputModule1Client.begin(analogInputModule1, 502)) {
      Serial.println("Analog Input Module 1 failed to connect!");
    } else {
      Serial.println("Analog Input Module 1 connected");
    }
  } 
  else
  {
    //------>Tarea 5
    //Atiende la interrupcion de MicroCuts
    chamber1.atiendeMicroCutsInterrup(flagAttacMicroCutsPointer); 
    //----------------------
    //---------> Tarea 5
    chamber1.atiendeGeneralSwitchDetect();
    //---------------------------
    chamber1.setaEmergency();
    //Aca se llama Todo el control
    chamber1.run();
  }

  //Ejecuta un interprete de la consola, para hacer debug
  //Funcion que ejecuta los comandos recibido del interprete o cosola debug
  tareaMainInterprete();
  //Ejecuta el interprete de las acciones de los comados
  interpreteEjecuta();
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

  if (timerGoOffAlarmTemperature > 0)
  {
    timerGoOffAlarmTemperature -= 1;
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

  if (timerLimitAlarmTemperature > 0)
  {
    timerLimitAlarmTemperature -= 1;
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

  if(timerAlarmNoVentilation > 0){
    timerAlarmNoVentilation -= 1;
  } else{
    timerAlarmNoVentilation = 0;
  }

  if(timerOpenDoorTimeAlarm1 > 0){
    timerOpenDoorTimeAlarm1 -= 1;
  } else{
    timerOpenDoorTimeAlarm1 = 0;
  }

  if(timerOpenDoorTimeAlarm2 > 0){
    timerOpenDoorTimeAlarm2 -= 1;
  } else{
    timerOpenDoorTimeAlarm2 = 0;
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



