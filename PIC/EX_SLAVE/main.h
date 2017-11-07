#include <16F1825.h>
#device ADC=10
#use delay(internal=32MHz)
#use rs232(baud=115200,parity=N,xmit=PIN_C4,rcv=PIN_C5,bits=8,stream=PORT1)
#use i2c(Slave,Fast,sda=PIN_C1,scl=PIN_C0,address=0xA0)

#define LED PIN_A5
#define DELAY 200


