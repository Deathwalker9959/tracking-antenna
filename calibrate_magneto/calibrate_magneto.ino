#include <Wire.h>
#include <Servo.h>
#ifdef USE_MPU9250
#include <MPU9250.h>
#define MPU9250_ADDRESS 0x68
MPU9250 mpu;
#else
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12346);
#endif

Servo elevation;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    elevation.attach(A1);
    elevation.write(10);
    while (!Serial)
        ;

#ifdef USE_MPU9250
    if (!mpu.setup(MPU9250_ADDRESS))
    {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        while (1)
        {
            delay(5000);
        }
    }
    Serial.println("MPU connection successful.");
#else
    if (!mag.begin())
    {
        Serial.println("LSM303 magnetometer connection failed. Please check your connection.");
        while (1)
        {
            delay(5000);
        }
    }
    if (!accel.begin())
    {
        Serial.println("LSM303 accelerometer connection failed. Please check your connection.");
        while (1)
        {
            delay(5000);
        }
    }
    Serial.println("LSM303 connection successful.");
#endif
}

void loop()
{
    static uint32_t prev_time = millis();
    
#ifdef USE_MPU9250
    if (mpu.update())
    {
        if (millis() - prev_time > 10)
        {
            Serial.print(String(mpu.getMagX()) + "," + String(mpu.getMagY()) + "," + String(mpu.getMagZ()) + ",");
            Serial.println(String(mpu.getAccX()) + "," + String(mpu.getAccY()) + "," + String(mpu.getAccZ()));
            prev_time = millis();
        }
    }
#else
    sensors_event_t mag_event, accel_event;
    elevation.write(10);
    mag.getEvent(&mag_event);
    accel.getEvent(&accel_event);
    if (millis() - prev_time > 10)
    {
        Serial.print(String(mag_event.magnetic.x) + "," + String(mag_event.magnetic.y) + "," + String(mag_event.magnetic.z) + ",");
        Serial.println(String(accel_event.acceleration.x) + "," + String(accel_event.acceleration.y) + "," + String(accel_event.acceleration.z));
        prev_time = millis();
    }
#endif
}
