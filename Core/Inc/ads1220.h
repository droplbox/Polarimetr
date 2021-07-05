#include "stm32f1xx.h"
#include "main.h"

#define ads1220_CS_LOW   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);     // Add your code here.
#define ads1220_CS_HIGH  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); 

void ads1220_reset(void);
void ads1220_sync(void);
void ads1220_pd(void);
uint32_t ads1220_rdata(void);
uint32_t ads1220_rreg(uint8_t r,uint8_t n);
void ads1220_wreg(uint8_t r,uint8_t n, uint32_t regdata);
void ads1220_init0(void);
void ads1220_init2(void);
void ads1220_deinit(void);
uint32_t  ads1220_getres(void);
void ads1220_chanset(uint8_t c);