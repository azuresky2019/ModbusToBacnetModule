#include <stdbool.h>
#include <stdint.h>

#define			MAX_PROFILE_LINE			255

typedef struct {
	bool		Poll;						/* yes no*/
	uint8_t * 	Bacnet_Object_Name;
	uint8_t		Modbus_Variable_Type;
	uint16_t	Modbus_Address;
	uint8_t		Data_Format;
	bool		bit;
	uint8_t		Low_Actul;
	uint8_t		High_Actul;
	uint8_t		Low_Scale;
	uint8_t		High_Scale;
	bool		ReadOnly_RW;
	uint8_t		Bacnet_Type;
	uint8_t *	Bacnet_Object_Description;
	uint8_t		COV_Increment;
	uint8_t		Unit_Group;
	uint8_t		Uint_Value;
	bool		Grouping;
	bool		Update_On_Reconnect;
		
}modbus_profile_t;



modbus_profile_t  modbus_profile[MAX_PROFILE_LINE];




