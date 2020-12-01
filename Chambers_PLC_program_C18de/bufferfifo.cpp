#include "bufferfifo.h"

bufferfifo::bufferfifo(/* args */)
{
    posIn = 0; 
    posOut = 0;
}

bufferfifo::~bufferfifo()
{
}

/**
 * @brief fillComandBuffer llena el buffer
 * @param command es el valor asociado a un comando que se guarda en el buffer interprete

*/

void bufferfifo::fillBuffer(unsigned int command){
    bufferOrdenesInterprete[posIn++] = command;
    
    if (posIn>=MAX_BUFFER_INTERPRTE)
    {
        posIn = 0;
    }
}

/**
 * @brief getcomandBuffer llena el buffer
 * @param command es un valor pon puntero, en el cual por el mismo se devolvera el valor
 * @return retonar true si hay data de interes, de lo contraio un false, por que no existe datos
*/

unsigned char bufferfifo::getcomandBuffer(){

    unsigned int command;

    command = bufferOrdenesInterprete[posOut++];

    if (posOut>=MAX_BUFFER_INTERPRTE)
    {
        posOut = 0;
    }

    return command;
    

}

char bufferfifo::statusBuffer(){
    return posOut != posIn;
}
