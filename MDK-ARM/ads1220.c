#include "stm32f1xx.h"
#include "ads1220.h"
SPI_HandleTypeDef hspi1;
void ads1220_reset(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_reset,1, HAL_TIMEOUT);
}
void ads1220_sync(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_sync,1, HAL_TIMEOUT);
}
void ads1220_pd(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_pd,1, HAL_TIMEOUT);
}
uint32_t ads1220_rdata(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_rdata,1, HAL_TIMEOUT);
	wfd=1;
	uint32_t data;
	uint8_t dat;
	for (uint8_t i=0; i++; i<3){
		data<<=8;
		HAL_SPI_Receive(&hspi1, &dat, 1, HAL_TIMEOUT);
		data|=dat;
	}
	return data;
}
uint32_t ads1220_rreg(uint8_t r,
											uint8_t n){
	uint8_t tdata;
	uint8_t rdat;
	uint32_t rdata;

	tdata |= ads1220_byte_rreg;
	tdata |= r<<2;
	tdata |= n;
	HAL_SPI_Transmit(&hspi1,&tdata,1, HAL_TIMEOUT);
	for(uint8_t i=0; i<n; i++){
		rdata<<=8;
		HAL_SPI_Receive(&hspi1,&rdat, 1, HAL_TIMEOUT);
		rdata|=rdat;
	}
	
	return rdata;
}
void ads1220_wreg(uint8_t r,
									uint8_t n,
									uint32_t regdata){
	uint8_t data;
	
	data|=ads1220_byte_wreg;
	data |= r<<2;
	data |= n;
	HAL_SPI_Transmit(&hspi1,&data,1, HAL_TIMEOUT);
	for(uint8_t i=0; i<n; i++){
		uint8_t tdat = regdata|0xF;
		HAL_SPI_Transmit(&hspi1, &tdat, 1, HAL_TIMEOUT);
		regdata>>=8;
	}
}
void ads1220_init(void){
	HAL_Delay(1);
	ads1220_CS_LOW;
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	ads1220_reset();
	HAL_Delay(100);
	ads1220_wreg(0,3,0x80041000);
	ads1220_sync();
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	ads1220_CS_HIGH;
}
void ads1220_deinit(void){
	ads1220_CS_LOW;
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	ads1220_pd();
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	ads1220_CS_HIGH;
}
uint32_t  ads1220_getres(void){
	uint32_t data;
	uint8_t dat;
	ads1220_CS_LOW;
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	for (uint8_t i=0; i<3; i++){
		data<<=8;
		HAL_SPI_Receive(&hspi1, &dat, 1, HAL_TIMEOUT);
		data|=dat;
	}
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	__asm("nop");
	ads1220_CS_HIGH;
	return data;
}