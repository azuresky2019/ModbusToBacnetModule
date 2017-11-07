#include "twi_master.h"
#include "twi_master_config.h"
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"



/******************************************************************************* 
 * ???? :   i2c_device_wirte_byte                                                                  
 * ??     :   wang                                                      
 *                                                                                
 * ?   ?  :  void                                                                    
 * ?   ?  :  void 
 * ?   ?  :  void                                                             
 * ???? : 20160317                                                                     
 *******************************************************************************/  
bool i2c_device_wirte_byte(uint8_t i2c_device_address, uint8_t  address, uint8_t data){  
     uint8_t   write_byte[2];  
         write_byte[0] = address;  
         write_byte[1] = data;  
         return  twi_master_transfer(i2c_device_address,write_byte,sizeof(write_byte),TWI_ISSUE_STOP);  
}  

/******************************************************************************* 
 * ???? :   i2c_device_wirte_data                                                                  
 * ??     :   wang                                                      
 *                                                                                
 * ?   ?  :  void                                                                    
 * ?   ?  :  void 
 * ?   ?  :  void                                                             
 * ???? : 20160317                                                                     
 *******************************************************************************/  
  
bool i2c_device_wirte_data(uint8_t i2c_device_address, uint8_t address, uint8_t *data, uint8_t lenght)  
{  
         return  twi_master_transfer(i2c_device_address,data,lenght,TWI_ISSUE_STOP);  
}  
/******************************************************************************* 
 * ???? :   i2c_device_read_byte                                                                  
 * ??     :   wang                                                      
 *                                                                                
 * ?   ?  :  void                                                                    
 * ?   ?  :  void 
 * ?   ?  :  void                                                             
 * ???? : 20160317                                                                     
 *******************************************************************************/  
uint8_t i2c_device_read_byte(uint8_t i2c_device_address, uint8_t address, uint8_t *p_read_byte, uint8_t length)  
{       bool i2c_trans_state  = false ;  
        i2c_trans_state = twi_master_transfer(i2c_device_address,&address,1,TWI_DONT_ISSUE_STOP);  
        while(i2c_trans_state == false);  
        i2c_trans_state =twi_master_transfer(i2c_device_address | TWI_READ_BIT,p_read_byte,length,TWI_ISSUE_STOP);  
            while(i2c_trans_state == false);   
  
          return  i2c_trans_state ; 
} 







