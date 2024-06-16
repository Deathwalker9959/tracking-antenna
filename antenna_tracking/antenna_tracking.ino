#include <Adafruit_GPS.h>
#include "secrets.h"
#include "helper.h"
#include "magneto.h"
#include "radio.h"
#include "servo.h"
#include <math.h>

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
GPSData gpsData;
Magneto imu; // Create an IMU object from the Magneto class

int targetServoAngle = 90; // Target servo position
float baseKp = 0.8;
float baseKi = 0.5;
float baseKd = 0.5;
float Kp, Ki, Kd;
float integral = 0;
float lastError = 0;
int errorSignChanges = 0;
unsigned long lastSignChangeTime = 0;
const unsigned long signChangeInterval = 3000; // 3 second interval to check for sign changes

const float maxKp = 1.0; // Maximum Kp value
const float minKp = 0.1; // Minimum Kp value
const float maxKi = 0.5; // Maximum Ki value
const float minKi = 0.05; // Minimum Ki value
const float maxKd = 1.0; // Maximum Kd value
const float minKd = 0.1; // Minimum Kd value

const float baseMaxAcceleration = 25.0; // Base max change in servo position per loop
const float minMaxAcceleration = 2.0; // Minimum max change in servo position per loop
const float maxMaxAcceleration = 20.0; // Maximum max change in servo position per loop
const float deadZone = 2.0; // Dead zone around the target angle to avoid oscillations
int elevationAngle = 10;   // Initial elevation position
bool elevationCompensationEnabled = false; // Toggle for elevation compensation

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Serial acquired");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // GPS.sendCommand(PGCMD_ANTENNA);

  initializeRadio();
  Serial.println("Radio acquired");

  initializeServo();
  Serial.println("Servo acquired");

  imu.setupIMU();
  imu.setKalmanIterations(3);
  Serial.println("IMU acquired");

  azimuth.write(targetServoAngle);
  elevation.write(elevationAngle);
  delay(1000);
  Kp = baseKp;
  Ki = baseKi;
  Kd = baseKd;

  Serial.println("System initialized.");
}

void loop()
{
  static uint32_t lastTime = 0;
  static int servoDelay = 0;
  static uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t buf_len = sizeof(buf);
  static float _lat, _lon, _decl, target_lat, target_lon, target_alt;

  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;

  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // Get IMU data
  Magneto::SensorData imuData = imu.getSensorData();

  if (rf95.recv(buf, &buf_len))
  {
    gpsData.decryptAndLoad(buf, buf_len, PSK);
    target_lat = gpsData.lat / 10000000.0;
    target_lon = gpsData.lon / 10000000.0;
    target_alt = gpsData.alt / 1000.0; // Assuming altitude is in millimeters
  }

  if (GPS.fix)
  {
    _lat = GPS.latitude_fixed / 10000000.0;
    _lon = GPS.longitude_fixed / 10000000.0;
    _decl = GPS.magvariation;
  }

  if (timeChange < 350)
    return;

  if (timeChange > servoDelay)
  {
    float currentHeading = imu.getBearing(imuData, _decl);
    // float targetHeading = calculateBearing(_lat, _lon, target_lat, target_lon);
    float targetHeading = 60;
    float error = targetHeading - currentHeading;

    error = fmod(error + 540, 360) - 180;

    if (lastError * error < 0)
    {
      errorSignChanges++;
      lastSignChangeTime = now;
    }

    if (errorSignChanges >= 3 && (now - lastSignChangeTime) < signChangeInterval)
    {
      Kp = constrain(Kp * 0.7, minKp, maxKp);
      Ki = constrain(Ki * 0.7, minKi, maxKi);
      Kd = constrain(Kd * 0.7, minKd, maxKd);
      errorSignChanges = 0; // Reset sign changes counter after applying damping
    }
    else if ((now - lastSignChangeTime) >= signChangeInterval)
    {
      Kp = min(Kp * 1.05, baseKp);
      Ki = min(Ki * 1.05, baseKi);
      Kd = min(Kd * 1.05, baseKd);
      Kp = constrain(Kp, minKp, maxKp);
      Ki = constrain(Ki, minKi, maxKi);
      Kd = constrain(Kd, minKd, maxKd);
      errorSignChanges = 0; // Reset sign changes counter after interval
    }

    if (abs(error) > deadZone)
    {
      // Calculate integral term
      integral += error * (timeChange / 1000.0);
      integral = constrain(integral, -100, 100);

      // Calculate derivative term
      float derivative = (error - lastError) / (timeChange / 1000.0);

      // PID output
      float output = Kp * error + Ki * integral + Kd * derivative;

      // Dynamic max acceleration based on proximity to target
      float dynamicMaxAcceleration = baseMaxAcceleration * (abs(error) / 66.0);
      dynamicMaxAcceleration = constrain(dynamicMaxAcceleration, minMaxAcceleration, maxMaxAcceleration);

      // Smooth transition to target angle
      if (abs(output) > dynamicMaxAcceleration)
      {
        output = (output > 0) ? dynamicMaxAcceleration : -dynamicMaxAcceleration;
      }

      targetServoAngle = constrain(targetServoAngle - output, 0, 180);

      moveServoToTargetSmooth(targetServoAngle);

      if (elevationCompensationEnabled)
      {
        // Calculate elevation compensation
        float distance = calculateDistance(_lat, _lon, target_lat, target_lon);
        elevationAngle = calculateElevationAngle(GPS.altitude / 1000.0, target_alt, distance);
        Serial.println("Distance: " + String(distance));
      }
      moveElevationTargetSmooth(elevationAngle);

      Serial.println("Current Heading: " + String(currentHeading) + " Target Heading: " + String(targetHeading) + " Error: " + String(error) + " Dynamic Kp: " + String(Kp) + " Dynamic Ki: " + String(Ki) + " Dynamic Kd: " + String(Kd) + " Output: " + String(output) + " Dynamic Max Acceleration: " + String(dynamicMaxAcceleration) + " Servo Angle: " + String(targetServoAngle) + " Declination: " + String(_decl) + " Elevation Angle: " + String(elevationAngle) + " Elevation Compensation: " + (elevationCompensationEnabled ? "Enabled" : "Disabled"));

      lastError = error;
    }
  }

  // Check for serial input to toggle elevation compensation or set custom elevation angle
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    if (input.length() > 0)
    {
      char command = input.charAt(0);
      if (command == 't') // Toggle command
      {
        elevationCompensationEnabled = !elevationCompensationEnabled;
        Serial.println("Elevation Compensation: " + String(elevationCompensationEnabled ? "Enabled" : "Disabled"));
      }
      else if (command == 'e') // Set custom elevation angle
      {
        int customAngle = input.substring(1).toInt();
        if (customAngle >= 0 && customAngle <= 90)
        {
          elevationAngle = customAngle;
          moveElevationTargetSmooth(elevationAngle);
          Serial.println("Custom Elevation Angle set to: " + String(elevationAngle));
        }
        else
        {
          Serial.println("Invalid Elevation Angle. Must be between 0 and 90.");
        }
      }
    }
  }

  lastTime = now;
}

void moveElevationTargetSmooth(int currentAngle)
{
  elevation.write(currentAngle);
}

void moveServoToTargetSmooth(int targetAngle)
{
  azimuth.write(targetAngle);
  delay(25); // Small delay to allow servo to move smoothly
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2)
{
  const float R = 6371e3; // Radius of the Earth in meters
  float phi1 = lat1 * PI / 180.0;
  float phi2 = lat2 * PI / 180.0;
  float deltaPhi = (lat2 - lat1) * PI / 180.0;
  float deltaLambda = (lon2 - lon1) * PI / 180.0;

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
            cos(phi1) * cos(phi2) *
            sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  float distance = R * c; // Distance in meters
  return distance;
}

int calculateElevationAngle(float currentAltitude, float targetAltitude, float distance)
{
  float deltaAltitude = targetAltitude - currentAltitude;
  float angle = atan2(deltaAltitude, distance) * 180.0 / PI;
  return constrain(angle, 0, 90); // Constrain the angle between 0 and 90 degrees
}
