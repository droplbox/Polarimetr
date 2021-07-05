# Polarimetr V0.1
 A simple polarimetr for IFA
Main board: Blue Pill (Stm32f103C8T6)
Sensor: Phototransistor TEMT6000 module
ADC: 24-bit 2ksps ADS1220 (TI) 
Display: 1602 via I2C convertor
Exciter: 450nm Diode 
Code partitially generated with STM32CubeMX

This device uses property of fluorescent molecules to rotate polarized light.

You need 3 pieces of 2x2mm polarizing glass, one in front of diode, 2 in front of sensors.