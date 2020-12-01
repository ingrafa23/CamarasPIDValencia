#include "consoladebug.h"
#include "bufferfifo.h"

bufferfifo fifoInteprete;

struct strdebug debugConsole = {0,0,0,0,0};
struct strdebug debugLastTime = {0,0,0,0,0};
{
    /* data */
};


void beginserial(){
}



//lista de instrucciones 
#define MAX_ORDENES 10
char *instrucciones[] = {"humidityon",                     //0
                         "humidityoff",                    //1
                         "ehtyleneoon",                    //2
                         "ethyleneoff",                    //3
                         "co2on",                          //4
                         "co2off",                         //5
                         "ethyleneflowon",                 //6
                         "ethyleneflowoff",                //7
                         "temperatureon",                  //8
                         "temperatureoff",                  //9
                         NULL};          




/**
 * @brief strEqual compara dos string
 * @param a el primer string a comparar
 * @param b el segundo string a comparar
 * @return retorna true si son iguales o false si son diferentes
*/

char strEqual(char *a, char *b)
{
    if (a == b)
        return 0xff;
    if (a == NULL || b == NULL)
        return 0x00;
    return strcmp(a, b) == 0;
}

/**
 * @brief interpreteIntrucciones
 * @param a el primer string a comparar
 * @param b el segundo string a comparar
 * @return retorna true si son iguales o false si son diferentes
*/

void interpreteIntrucciones(char *commandIntrucciones){
        
    for (unsigned int i = 0; i < MAX_ORDENES; i++)
    {
        /* code */
        if(strEqual(commandIntrucciones,instrucciones[i])){
            fifoInteprete.fillBuffer(i);
            
            break;
        }
        
    }
    
}


void interpreteEjecuta(){

    
    if(fifoInteprete.statusBuffer()){
        
        unsigned char command = fifoInteprete.getcomandBuffer();

        Serial.print("Comando : ");Serial.println(command);

        Serial.println(instrucciones[command]);
    
        switch (command)
        {
            case 0:
                debugConsole.humidity = 1;
                break;
            
            case 1:
                debugConsole.humidity = 0;
                break;
            
             case 2:
                debugConsole.ethylene = 1;
                break;

            case 3:
                debugConsole.ethylene = 0;
                break;

            case 4:
                debugConsole.ethyleneFlow = 1;
                break;

            case 5:
                debugConsole.ethyleneFlow = 0;
                break;

            case 6:
                debugConsole.co2 = 1;
                break;
            
            case 7:
                debugConsole.co2 = 0;
                break;
                
            case 8:
                debugConsole.temp = 1;
                break;
            case 9:
                debugConsole.temp = 0;
                break;
            
            default:
                break;
        }
    
    }

}

void tareaMainInterprete(){
    if(Serial.available()>0){
        char data_recibida[64];
        for (unsigned char i = 0; i < 64; i++)
        {
            if (Serial.available()>0){
                data_recibida[i] = (char)Serial.read();     
            }
            else
            {
                data_recibida[i]='\0';
            } 
        }
        Serial.println(data_recibida);
        interpreteIntrucciones(data_recibida);
    }  
}