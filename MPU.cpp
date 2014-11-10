#include "MPU.h"
#include "Arduino.h"


extern "C" int i2c_write(unsigned char slave_addr,
                         unsigned char reg_addr,
                         unsigned char length,
                         unsigned char const *data)
{
    
}

extern "C" int i2c_read(unsigned char slave_addr,
                        unsigned char reg_addr,
                        unsigned char length,
                        unsigned char *data)
{
    
}

extern "C" void delay_ms(unsigned long num_ms)
{
    delay(num_ms);
}

extern "C" void get_ms(unsigned long *count)
{
    *count = millis();
}
