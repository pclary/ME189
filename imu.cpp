#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
//#include "i2c_t3.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "utilities.h"


volatile long quat[4];
volatile int cal_samples_remaining;
const int num_cal_samples = 64;
volatile int64_t gyro_accumulator[3];
volatile int64_t accel_accumulator[3];

static inline unsigned short inv_row_2_scale(const signed char *row);
static inline unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);


void imu_init()
{
    //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
    Wire.begin();
    
    int_param_s int_param;
    int_param.pin = 17;
    int_param.mode = FALLING;
    int_param.cb = imu_int_callback;
    const signed char gyro_orientation[9] = { 1,  0,  0,
                                              0,  1,  0,
                                              0,  0,  1};

    if (mpu_init(&int_param))
        led_on();
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(200);
    
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT |
                       DMP_FEATURE_SEND_RAW_ACCEL |
                       DMP_FEATURE_SEND_RAW_GYRO |
                       DMP_FEATURE_GYRO_CAL |
                       DMP_FEATURE_TAP);
    dmp_set_fifo_rate(200);
    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
    mpu_set_dmp_state(1);

    led_off();
    delay(100);
    led_on();

    delay(500);

    cal_samples_remaining = num_cal_samples;
    for (int i = 0; i < 3; ++i)
    {
        gyro_accumulator[i] = 0;
        accel_accumulator[i] = 0;
    }
    while (cal_samples_remaining > 0);
    
    long gyro_bias[3];
    long accel_bias[3];
    for (int i = 0; i < 3; ++i)
    {
        gyro_bias[i] /= num_cal_samples;
        accel_bias[i] /= num_cal_samples;
    }

    char buf[256];
    snprintf(buf, 256, "newacc:  %d %d %d\nnewgyro: %d %d %d\n", accel_bias[0], accel_bias[1], accel_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    Serial.print(buf);

    
    snprintf(buf, 256, "oldacc:  %d %d %d\noldgyro: %d %d %d\n", accel_bias[0], accel_bias[1], accel_bias[2], gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    Serial.print(buf);
}


int i2c_write(unsigned char slave_addr,
              unsigned char reg_addr,
              unsigned char length,
              unsigned char const *data)
{
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    Wire.write(data, length);
    Wire.endTransmission();

    return 0;
}


int i2c_read(unsigned char slave_addr,
             unsigned char reg_addr,
             unsigned char length,
             unsigned char *data)
{
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(slave_addr, (size_t)length);
    for (int i = 0; i < length; ++i)
        data[i] = Wire.read() & 0xff;
    
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


int reg_int_cb(struct int_param_s *int_param)
{
    pinMode(int_param->pin, INPUT);
    attachInterrupt(int_param->pin, int_param->cb, int_param->mode);
    return 0;
}


void imu_int_callback()
{
    short gyro[3], accel[3], sensors;
    unsigned char more;
    unsigned long sensor_timestamp;

    dmp_read_fifo(gyro, accel, (long*)quat, &sensor_timestamp, &sensors, &more);

    if (cal_samples_remaining > 0)
    {
        for (int i = 0; i < 3; ++i)
        {
            gyro_accumulator[i] += gyro[i];
            accel_accumulator[i] += accel[i];
        }
        
        --cal_samples_remaining;
    }
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
