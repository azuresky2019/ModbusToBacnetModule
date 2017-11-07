#include <16F1825.h>
#device ADC=10
#use delay(internal=32MHz,restart_wdt)
#use i2c(Slave,Fast,sda=PIN_C1,scl=PIN_C0,address=0xA0)
#define LED PIN_C2
#define DELAY 1000


