#include "stm32f1xx.h"
#include "main.h"

uint8_t ads1220_byte_reset =3<<1;
uint8_t ads1220_byte_sync = 1<<3;
uint8_t ads1220_byte_pd = 1<<1;
uint8_t ads1220_byte_rdata = 1<<4;
uint8_t ads1220_byte_rreg = 1<<5;
uint8_t ads1220_byte_wreg = 1<<6;
uint8_t wfd =0;
#define ads1220_CS_LOW   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);     // Add your code here.
#define ads1220_CS_HIGH  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); 

void ads1220_reset(void);
void ads1220_sync(void);
void ads1220_pd(void);
uint32_t ads1220_rdata(void);
uint32_t ads1220_rreg(uint8_t r,uint8_t n);
void ads1220_wreg(uint8_t r,uint8_t n, uint32_t regdata);
void ads1220_init(void);
void ads1220_deinit(void);