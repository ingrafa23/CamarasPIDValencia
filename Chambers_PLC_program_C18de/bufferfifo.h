#ifndef BUFFER_FIFO_INCLUDE
#define BUFFER_FIFO_INCLUDE

#include <string.h>
#include <Arduino.h>

#define MAX_BUFFER_INTERPRTE  16

class bufferfifo
{
private:
    /* data */
    

    unsigned int bufferOrdenesInterprete[MAX_BUFFER_INTERPRTE];
    unsigned int posIn; 
    unsigned int posOut;

public:
    bufferfifo(/* args */);
    ~bufferfifo();

    void fillBuffer(unsigned int command);
    unsigned char getcomandBuffer();
    char statusBuffer();
};




#endif