#include <main.h>

#include <2404.C>

void main()
{

   BYTE value, cmd;
   EEPROM_ADDRESS address;
   
   init_ext_eeprom();

   while(TRUE)
   {      
      
      address = 1;

      value = READ_EXT_EEPROM( address );
      printf("\r\nValue: %X\r\n", value);      
      memset(value,0x00,sizeof(value));
      
      //Example blinking LED program
      output_low(LED);
      delay_ms(DELAY);
      output_high(LED);
      delay_ms(DELAY);

      //TODO: User Code
   }

}
