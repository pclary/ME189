#ifndef MPU_H
#define MPU_H

#ifdef __cplusplus
extern "C"
{
#endif

int i2c_write(unsigned char slave_addr,
              unsigned char reg_addr,
              unsigned char length,
              unsigned char const *data);

int i2c_read(unsigned char slave_addr,
             unsigned char reg_addr,
             unsigned char length,
             unsigned char *data);

void delay_ms(unsigned long num_ms);

void get_ms(unsigned long *count);

#ifdef __cplusplus
}
#endif

#endif // MPU_H
