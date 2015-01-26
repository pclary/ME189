#include <Arduino.h>
#include <SPI.h>
#include "imu.h"
#include "RingBuffer.h"


struct LogData
{
    long qw, qx, qy, qz;
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
        dataLog.push({quat[0], quat[1], quat[2], quat[3]});
    }
}


void loop()
{
    while (!Serial.available());

    while (Serial.read() != -1);

    char buf[256];

    for (int i = 0; i < dataLog.size(); ++i)
    {
        snprintf(buf, 256, "%d,%d,%d,%d\n", dataLog[i].qw, dataLog[i].qx, dataLog[i].qy, dataLog[i].qz);
        Serial.write(buf);
    }
}
