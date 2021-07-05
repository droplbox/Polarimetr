#include "stm32f1xx.h"
#include "ads1220.h"
uint8_t ads1220_byte_reset =3<<1;
uint8_t ads1220_byte_sync = 1<<3;
uint8_t ads1220_byte_pd = 1<<1;
uint8_t ads1220_byte_rdata = 1<<4;
uint8_t ads1220_byte_rreg = 1<<5;
uint8_t ads1220_byte_wreg = 1<<6;
uint8_t wfd =0;
extern SPI_HandleTypeDef hspi1;
void ads1220_reset(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_reset,1, HAL_TIMEOUT);
}
void ads1220_sync(void){	ads1220_CS_LOW;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_sync,1, HAL_TIMEOUT);
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_CS_HIGH;
	
}
void ads1220_pd(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_pd,1, HAL_TIMEOUT);
}
uint32_t ads1220_rdata(void){
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_rdata,1, HAL_TIMEOUT);
	wfd=1;
	uint32_t data=0;
	uint8_t dat=0;
	for (uint8_t i=0; i<3; i++){
		data<<=8;
		HAL_SPI_Receive(&hspi1, &dat, 1, HAL_TIMEOUT);
		data|=dat;
	}
	return data;
}
uint32_t ads1220_rreg(uint8_t r,
											uint8_t n){
	uint8_t tdata=0;
	uint8_t rdat=0;
	uint32_t rdata=0;

	tdata |= ads1220_byte_rreg;
	tdata |= r<<2;
	tdata |= n;
	HAL_SPI_Transmit(&hspi1,&tdata,1, HAL_TIMEOUT);
	HAL_Delay(1);
	for(uint8_t i=0; i<=n; i++){
		rdata<<=8;
		HAL_SPI_Receive(&hspi1,&rdat, 1, HAL_TIMEOUT);
		rdata|=rdat;
	}
	
	return rdata;
}
void ads1220_wreg(uint8_t r,
									uint8_t n,
									uint32_t regdata){
	uint8_t data=0;
	uint8_t tdat=0;
	
	data|=ads1220_byte_wreg;
	data |= r<<2;
	data |= n;
	HAL_SPI_Transmit(&hspi1,&data,1, HAL_TIMEOUT);
	HAL_Delay(1);
	for(uint8_t i=n; i!=0; i--){
		tdat = (regdata>>(8*i))&0xFF;
		HAL_SPI_Transmit(&hspi1, &tdat, 1, HAL_TIMEOUT);
	}
}
void ads1220_init0(void){
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_CS_LOW;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_reset();
	HAL_Delay(100);
	ads1220_wreg(0,3,0x80001000);
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_sync();
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_CS_HIGH;
}
void ads1220_init2(void){
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_CS_LOW;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_reset();
	HAL_Delay(100);
	ads1220_wreg(0,3,0xA0001000);
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_sync();
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_CS_HIGH;
}
void ads1220_deinit(void){
	ads1220_CS_LOW;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_pd();
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	
	ads1220_CS_HIGH;
}
uint32_t  ads1220_getres(void){
	uint32_t data=0;
	uint8_t dat=0;
	ads1220_CS_LOW;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	for (uint8_t i=0; i<3; i++){
		data<<=8;
		HAL_SPI_Receive(&hspi1, &dat, 1, HAL_TIMEOUT);
		data|=dat;
	}
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	ads1220_CS_HIGH;
	return data;
}
void ads1220_chanset(uint8_t c){
	uint8_t dat=0;
	ads1220_CS_LOW;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	dat = ads1220_rreg(0,0);
	c|=0x8;
	c<<=4;
	dat&=0xF;
	dat|=c;
	ads1220_wreg(0,0,dat);
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");	
	__asm("NOP");
	__asm("NOP");
	ads1220_CS_HIGH;
}