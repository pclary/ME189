#include <Arduino.h>
#include <SPI.h>
#include "imu.h"
#include "RingBuffer.h"


struct LogData
{
    short ax, ay, az, gx, gy, gz;
    unsigned long ts;
};

RingBuffer<LogData, 3072> dataLog;


void setup()
{
    Serial.begin(115200);

    delay(100);
    imu_init();

    delay(10000);

    while (dataLog.size() < dataLog.capacity())
    {
        while (!imu_check_update());
        dataLog.push({accel[0], accel[1], accel[2],
                    gyro[0], gyro[1], gyro[2], sensor_timestamp});
    }
}


void loop()
{
    while (!Serial.available());

    while (Serial.read() != -1);

    char buf[256];

    for (int i = 0; i < dataLog.size(); ++i)
    {
        snprintf(buf, 256, "%d,%d,%d,%d,%d,%d\n", dataLog[i].ax, dataLog[i].ay, dataLog[i].az, dataLog[i].gx, dataLog[i].gy, dataLog[i].gz);
        Serial.write(buf);
    }
}
