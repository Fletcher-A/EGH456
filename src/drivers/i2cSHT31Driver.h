/**************************************************************************************************
*  Filename:       i2cSHT31Driver.h
*
*  Description:    i2c Driver for use with the SHT temperature and humidity Sensor
*************************************************************************************************/



#ifndef _I2COPTDRIVER_H_
#define _I2COPTDRIVER_H_



// ----------------------- Includes -----------------------
#include <stdbool.h>
#include <stdint.h>



// ----------------------- Exported prototypes -----------------------
extern void initI2C2(uint32_t ui32SysClock);
extern bool writeI2C2(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data);
extern bool writeI2C2Single(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t  Data);
extern bool readSHT31(uint8_t ui8Addr, uint8_t *data);



#endif /* _I2COPTDRIVER_H_ */
