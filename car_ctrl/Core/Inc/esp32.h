#ifndef __esp32_H
#define __esp32_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"

typedef struct
{
	uint8_t mode ;
	uint8_t flash_ok ;
	
}esp32_t ;
	 
void ESP32ModeCtrl( uint8_t state);
void ESP32_Start( uint32_t mode );
void ESP32_Init( void );
void ESP32_SendData(uint8_t* data, uint16_t size);
void ESP32_ReceiveData(uint8_t* buffer, uint16_t size);
void check_esp32_at_mode(void);

extern esp32_t g_esp32 ;

#ifdef __cplusplus
}
#endif
#endif /*__esp32_H */
