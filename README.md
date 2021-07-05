# Polarimetr V0.1
 A simple polarimetr for PFIA
Main board: Blue Pill (Stm32f103C8T6)
Sensor: Phototransistor TEMT6000 module
ADC: 24-bit 2ksps ADS1220 (TI) 
Display: 1602 via I2C convertor
Exciter: 405nm Diode 
Code partitially generated with STM32CubeMX

This device uses property of fluorescent molecules to rotate polarized light.

You need 3 pieces of 3x3mm polarizing glass, one in front of diode, 2 in front of sensors.