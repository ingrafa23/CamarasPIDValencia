#include <Ethernet.h>
#include <ArduinoModbus.h>
#include "miscellaneous.h"
#include "controlChambers.h"
#include "variablesAlarm.h"
#include "variablestimer.h"


#define GENERAL_SWITCH_DETEC  I1_4  //Detección de interruptor principal
#define MICRO_CUTS_DETECTION  I1_5  //Entrada relé para la detección de microcortes
#define EMERGENCY_STOP        I1_6  //Entrada relé emergencia      

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

int humidityInyectionTimes[2] = {0, 0};
int* humidityInyectionTimesPointer = &humidityInyectionTimes[0];

bool humidityInyectionStatus = 0;
bool* humidityInyectionStatusPointer = &humidityInyectionStatus;

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

int timerGoOffAlarmhumidity = 0;
int* timerGoOffAlarmHumidityPointer = &timerGoOffAlarmhumidity;

int timerGoOffAlarmEthylene = 0;
int* timerGoOffAlarmEthylenePointer = &timerGoOffAlarmEthylene;

int timerGoOffAlarmCO2 = 0;
int* timerGoOffAlarmCO2Pointer = &timerGoOffAlarmCO2;

int timerLimitAlarmTemperature = 0;
int* timerLimitAlarmTemperaturePointer = &timerLimitAlarmTemperature;

int timerLimitAlarmHumidity = 0;
int* timerLimitAlarmHumidityPointer = &timerLimitAlarmHumidity;

int timerLimitAlarmEthylene = 0;
int* timerLimitAlarmEthylenePointer = &timerLimitAlarmEthylene;

int timerLimitAlarmCO2 = 0;
int* timerLimitAlarmCO2Pointer = &timerLimitAlarmCO2;

int timerInitializationFan = 0;
int* timerInitializationFanPointer = &timerInitializationFan;

int timerAlarmNoVentilation;
int *timerAlarmNoVentilationPointer = &timerAlarmNoVentilation;

int timerOpenDoorTimeAlarm1;
int *timerOpenDoorTimeAlarm1Pointer = &timerOpenDoorTimeAlarm1;

int timerOpenDoorTimeAlarm2;
int *timerOpenDoorTimeAlarm2Pointer = &timerOpenDoorTimeAlarm2;


int delayConnection;

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

  attachInterrupt(digitalPinToInterrupt(MICRO_CUTS_DETECTION), microCuts, FALLING);

  
  chamber1.init(timerAlarmNoVentilationPointer,
                  timerOpenDoorTimeAlarm1Pointer,
                  timerOpenDoorTimeAlarm2Pointer);

}

unsigned int numeroPasos = 0;

void loop() {

  //Serial.print("Timer etileno : "); Serial.println(*(ethyleneInyectionTimesPointer + *ethyleneInyectionStatusPointer));
  

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
                  timerGoOffAlarmCO2Pointer,
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
    //enable control
    chamber1.enableControl();
    //forced control
    chamber1.forcedControl();
    
    chamber1.writeAnalogValues();
  }
  

}

void microCuts()
{
  if (!digitalRead(GENERAL_SWITCH_DETEC))
  {
    setBitEeprom(0, 3); // Seteamos el bit en eeprom para dejar constancia del microcorte
  }
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

  if (*timerGoOffAlarmHumidityPointer > 0)
  {
    *timerGoOffAlarmHumidityPointer -= 1;
  }

  if (*timerGoOffAlarmEthylenePointer > 0)
  {
    *timerGoOffAlarmEthylenePointer -= 1;
  }

  if (*timerGoOffAlarmCO2Pointer > 0)
  {
    *timerGoOffAlarmCO2Pointer -= 1;
  }

  if (*timerLimitAlarmTemperaturePointer > 0)
  {
    *timerLimitAlarmTemperaturePointer -= 1;
  }

  if (*timerLimitAlarmHumidityPointer > 0)
  {
    *timerLimitAlarmHumidityPointer -= 1;
  }

  if (*timerLimitAlarmEthylenePointer > 0)
  {
    *timerLimitAlarmEthylenePointer -= 1;
  }

  if (*timerLimitAlarmCO2Pointer > 0)
  {
    *timerLimitAlarmCO2Pointer -= 1;
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
  
  
  



  

  TCNT5H = 0xC2;
  TCNT5L = 0xF7;
}
