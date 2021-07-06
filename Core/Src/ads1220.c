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
void ads1220_sync(void){ // transmits sync frame	
	ads1220_CS_LOW;
	HAL_Delay(5);
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_sync,1, HAL_TIMEOUT);
	HAL_Delay(5);
	ads1220_CS_HIGH;
	
}
void ads1220_pd(void){ // powerdown(not realized)
	HAL_SPI_Transmit(&hspi1,&ads1220_byte_pd,1, HAL_TIMEOUT);
}
uint32_t ads1220_rdata(void){ //reading data. use only CS tied low
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
											uint8_t n){ //reading from register r n bytes. use only CS tied low
	uint8_t tdata=0;
	uint8_t rdat=0;
	uint32_t rdata=0;

	tdata |= ads1220_byte_rreg;
	tdata |= r<<2;
	tdata |= n;

	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1,&tdata,1, HAL_TIMEOUT);

	for(uint8_t i=0; i<=n; i++){ 
		rdata<<=8;
		HAL_SPI_Receive(&hspi1,&rdat, 1, HAL_TIMEOUT);
		rdata|=rdat;
	}
	HAL_Delay(1);

	return rdata;
}
void ads1220_wreg(uint8_t r,
									uint8_t n,
									uint32_t regdata){ //writing to register r n bytes from regdata. use only CS tied low
	uint8_t data=0;
	uint8_t tdat=0;
	
	data|=ads1220_byte_wreg;
	data |= r<<2;
	data |= n;

	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi1,&data,1, HAL_TIMEOUT);
	for(uint8_t i=n; i!=0; i--){
		tdat = (regdata>>(8*i))&0xFF;
		HAL_SPI_Transmit(&hspi1, &tdat, 1, HAL_TIMEOUT);
	}
	HAL_Delay(5);

}
void ads1220_init(void){ // init with reading from ch0
	HAL_Delay(5);
	ads1220_CS_LOW;
	HAL_Delay(5);
	ads1220_reset();
	HAL_Delay(100);
	ads1220_wreg(0,3,0x80001000);
	HAL_Delay(5);
	ads1220_sync();
	ads1220_CS_HIGH;
}
void ads1220_init2(void){ //init with reading from ch2
	HAL_Delay(5);
	ads1220_CS_LOW;
	HAL_Delay(5);
	ads1220_reset();
	HAL_Delay(100);
	ads1220_wreg(0,3,0xA0001000);
	HAL_Delay(5);
	ads1220_sync();
	HAL_Delay(5);
	ads1220_CS_HIGH;
}
void ads1220_deinit(void){ //deinitialize
	ads1220_CS_LOW;
	HAL_Delay(1);
	ads1220_pd();
	HAL_Delay(1);
	ads1220_CS_HIGH;
}
uint32_t  ads1220_getres(void){ //reading results
	uint32_t data=0;
	uint8_t dat=0;
	ads1220_CS_LOW;
	HAL_Delay(5);
	for (uint8_t i=0; i<3; i++){
		data<<=8;
		HAL_SPI_Receive(&hspi1, &dat, 1, HAL_TIMEOUT);
		data|=dat;
	}
	HAL_Delay(5);
	ads1220_CS_HIGH;
	return data;
}
void ads1220_chanset(uint8_t c){ //selecting channels
	uint8_t dat=0;
	ads1220_CS_LOW;
	HAL_Delay(5);
	if (c==0){
	dat = 0b10000001;
	} else if (c==2){
	dat = 0b10100001;
	}
	ads1220_wreg(0,1,dat);
	HAL_Delay(50);
	ads1220_CS_HIGH;
}