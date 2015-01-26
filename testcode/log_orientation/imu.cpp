#include "imu.h"
#include <Arduino.h>
#include <SPI.h>
#include <stdio.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


const int interruptPin = 7;
const int ncsPin = 10;
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);

volatile long quat[4];
volatile char new_update = false;

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);


void imu_init()
{
    pinMode(ncsPin, OUTPUT);
    digitalWrite(ncsPin, HIGH);
    SPI.begin();
    
    int_param_s int_param;
    int_param.pin = interruptPin;
    int_param.mode = FALLING;
    int_param.cb = imu_int_callback;
    const signed char gyro_orientation[9] = { 1,  0,  0,
                                              0,  1,  0,
                                              0,  0,  1};

    mpu_init(&int_param); // 153.27 ms
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); // 50.03 ms
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); // 50.16 ms
    mpu_set_sample_rate(200); // 0.044 ms

    dmp_load_motion_driver_firmware(); // 1.02 ms
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)); // 0.28 ms
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |
                       DMP_FEATURE_GYRO_CAL |
                       DMP_FEATURE_TAP); // 151.86 ms
    
    dmp_set_fifo_rate(200); // 0.212 ms
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS); // 0.142 ms
    mpu_set_dmp_state(1); // 0.002 ms
}


/* Uses SPI instead of I2C */
int i2c_write(unsigned char slave_addr,
              unsigned char reg_addr,
              unsigned char length,
              unsigned char const *data)
{

    SPI.beginTransaction(spiSettings);
    digitalWrite(ncsPin, LOW);
    SPI.transfer(reg_addr);
    for (int i = 0; i < length; ++i)
        SPI.transfer(data[i]);
    digitalWrite(ncsPin, HIGH);
    SPI.endTransaction();

    return 0;
}


/* Uses SPI instead of I2C */
int i2c_read(unsigned char slave_addr,
             unsigned char reg_addr,
             unsigned char length,
             unsigned char *data)
{
    SPI.beginTransaction(spiSettings);
    digitalWrite(ncsPin, LOW);
    SPI.transfer(reg_addr | 0x80);
    for (int i = 0; i < length; ++i)
        data[i] = SPI.transfer(0) & 0xff;
    digitalWrite(ncsPin, HIGH);
    SPI.endTransaction();
    
    return 0;
}


void delay_ms(unsigned long num_ms)
{
    delay(num_ms);
}


void get_ms(unsigned long *count)
{
    *count = millis();
}


void log_serial(const char* format, ...)
{
    char buf[256];
    va_list argptr;
    va_start(argptr, format);
    vsnprintf(buf, 256, format, argptr);
    va_end(argptr);
    Serial.print(buf);
}


int reg_int_cb(struct int_param_s *int_param)
{
    pinMode(int_param->pin, INPUT);
    attachInterrupt(int_param->pin, int_param->cb, int_param->mode);
    SPI.usingInterrupt(int_param->pin);
    return 0;
}


void imu_int_callback()
{
    short gyro[3], accel[3], sensors;
    unsigned char more;
    unsigned long sensor_timestamp;

    dmp_read_fifo(gyro, accel, (long*)quat, &sensor_timestamp, &sensors, &more);
    new_update = 1;
}


char imu_check_update()
{
    if (new_update)
    {
        new_update = 0;
        return 1;
    }

    return 0;
}


/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
