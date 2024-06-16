#ifndef MAGNETO_H
#define MAGNETO_H

#include <Wire.h>
#include <math.h> // Include this for mathematical constants and functions
#include "kalman.h"
#ifdef USE_MPU9250
#include <MPU9250.h>
#define SENSOR_ADDRESS 0x68
#else
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#endif

class Magneto
{
public:
    struct SensorData
    {
        float magX;
        float magY;
        float magZ;
        float accX;
        float accY;
        float accZ;
    };

    void setupIMU();
    SensorData getSensorData();
    float getBearing(SensorData data, float declination);

    void setKalmanIterations(int iterations); // Setter method for Kalman iterations

private:
#ifdef USE_MPU9250
    MPU9250 mpu;
#else
    Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12346);
#endif

    void applyMagnetometerCalibration(SensorData &data);
    void applyAccelerometerCalibration(SensorData &data);
    const float magSoftIronMatrix[3][3] = {
        {1.2365064398063066, 0.015304826501342173, 0.029863865824864638},
        {0.015304826501342227, 1.21675728185341, 0.0001089229010195789},
        {0.029863865824864607, 0.00010892290101954505, 1.2695473068743899}};

    const float magHardIronBias[3] = {
        -11.417768759721355, -0.4049487161097772, 14.033909422714439};

    const float accelHardIronBias[3] = {
        -0.7707038728504586, 1.0349540408946096, -0.27965574347220523};

    KalmanFilter kalmanX;
    KalmanFilter kalmanY;
    KalmanFilter kalmanZ;

    int kalmanIterations; // Member variable for Kalman iterations count
};

void Magneto::setupIMU()
{
#ifdef USE_MPU9250
    if (!mpu.setup(SENSOR_ADDRESS))
    {
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        while (1)
        {
            delay(5000);
        }
    }

    mpu.setAccBias(-62.08, 48.22, -55.37);
    mpu.setGyroBias(6.01, -1.24, -2.97);
    mpu.setMagBias(178.508292, -450.428962, -1160.172018);
    mpu.setMagScale(1.149643, 1.005773, 0.866209);
    mpu.setMagneticDeclination(0);
    mpu.selectFilter(QuatFilterSel::MADGWICK);
    mpu.setFilterIterations(2);
#else
    mag.enableAutoRange(true);
    mag.setMagRate(LSM303_MAGRATE_220);
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

    kalmanX.init(0, 1);
    kalmanY.init(0, 1);
    kalmanZ.init(0, 1);

    kalmanIterations = 1; // Default value for Kalman iterations
}

void Magneto::setKalmanIterations(int iterations)
{
    kalmanIterations = iterations;
}

Magneto::SensorData Magneto::getSensorData()
{
    SensorData data;
#ifdef USE_MPU9250
    data.magX = mpu.getMagX();
    data.magY = mpu.getMagY();
    data.magZ = mpu.getMagZ();
    data.accX = mpu.getAccX();
    data.accY = mpu.getAccY();
    data.accZ = mpu.getAccZ();
#else
    sensors_event_t magEvent, accelEvent;
    mag.getEvent(&magEvent);
    accel.getEvent(&accelEvent);

    data.magX = magEvent.magnetic.x;
    data.magY = magEvent.magnetic.y;
    data.magZ = magEvent.magnetic.z;
    data.accX = accelEvent.acceleration.x;
    data.accY = accelEvent.acceleration.y;
    data.accZ = accelEvent.acceleration.z;

    applyMagnetometerCalibration(data);
    applyAccelerometerCalibration(data);

    // Apply Kalman filter
    for (int i = 0; i < kalmanIterations; i++)
    {
        data.magX = kalmanX.update(data.magX);
        data.magY = kalmanY.update(data.magY);
        data.magZ = kalmanZ.update(data.magZ);
    }
#endif
    return data;
}

void Magneto::applyMagnetometerCalibration(SensorData &data)
{
    float calibratedMagX = magSoftIronMatrix[0][0] * (data.magX - magHardIronBias[0]) +
                           magSoftIronMatrix[0][1] * (data.magY - magHardIronBias[1]) +
                           magSoftIronMatrix[0][2] * (data.magZ - magHardIronBias[2]);

    float calibratedMagY = magSoftIronMatrix[1][0] * (data.magX - magHardIronBias[0]) +
                           magSoftIronMatrix[1][1] * (data.magY - magHardIronBias[1]) +
                           magSoftIronMatrix[1][2] * (data.magZ - magHardIronBias[2]);

    float calibratedMagZ = magSoftIronMatrix[2][0] * (data.magX - magHardIronBias[0]) +
                           magSoftIronMatrix[2][1] * (data.magY - magHardIronBias[1]) +
                           magSoftIronMatrix[2][2] * (data.magZ - magHardIronBias[2]);

    data.magX = calibratedMagX;
    data.magY = calibratedMagY;
    data.magZ = calibratedMagZ;
}

void Magneto::applyAccelerometerCalibration(SensorData &data)
{
    float calibratedAccX = data.accX - accelHardIronBias[0];
    float calibratedAccY = data.accY - accelHardIronBias[1];
    float calibratedAccZ = data.accZ - accelHardIronBias[2];

    data.accX = calibratedAccX;
    data.accY = calibratedAccY;
    data.accZ = calibratedAccZ;
}

float Magneto::getBearing(SensorData data, float declination)
{
    // Calculate pitch and roll
    float roll = atan2(data.accY, data.accZ);                                             // Roll calculation using Y-axis as 'left'
    float pitch = atan2(-data.accX, sqrt(data.accY * data.accY + data.accZ * data.accZ)); // Pitch calculation using X-axis as 'forward'

    // Tilt compensation for magnetic field readings
    float magXComp = data.magX * cos(pitch) + data.magZ * sin(pitch);                                                 // magX as forward vector
    float magYComp = data.magX * sin(roll) * sin(pitch) + data.magY * cos(roll) - data.magZ * sin(roll) * cos(pitch); // Adjusting compensation

    // Calculate heading
    float heading = atan2(magYComp, magXComp) * 180.0 / PI; // Adjusted to match the correct forward vector
    heading += declination;

    // Normalize heading to 0-360°
    if (heading < 0)
        heading += 360;
    if (heading >= 360)
        heading -= 360;

    return heading;
}

#endif // MAGNETO_H