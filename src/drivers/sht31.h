/*
 * sht31.h — SHT31 temperature & humidity I2C sensor (optional).
 *
 * Default I2C address: 0x44 (ADDR pin low) or 0x45 (ADDR pin high).
 */
#ifndef SHT31_H
#define SHT31_H

#include <stdbool.h>

bool  sht31_init(void);
bool  sht31_read(float *temp_c, float *humidity_rh);

#endif /* SHT31_H */
