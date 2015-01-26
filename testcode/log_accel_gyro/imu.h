#ifndef MPU_H
#define MPU_H

#ifdef __cplusplus
extern "C"
{
#endif
 
extern volatile short accel[3];
extern volatile short gyro[3];
extern volatile unsigned long sensor_timestamp;
    
void imu_init();
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
void log_serial(const char* format, ...);
int reg_int_cb(struct int_param_s *int_param);
void imu_int_callback();
char imu_check_update();

#ifdef __cplusplus
}
#endif

#endif // MPU_H
