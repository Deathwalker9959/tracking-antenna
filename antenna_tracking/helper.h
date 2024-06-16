// helper.h
#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <string.h>  // Include C string functions
#include <stdint.h>  // Include standard integer types

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif


// Function to encrypt or decrypt data

void xorEncrypt(uint8_t* data, size_t len, const char* key) {
  size_t keyLen = strlen(key);
  for (size_t i = 0; i < len; i++) {
    data[i] ^= key[i % keyLen];
  }
}

// Function to convert degrees to radians
double degToRad(double deg) {
  return deg * (M_PI / 180.0);
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    lat1 = degToRad(lat1);
    lon1 = degToRad(lon1);
    lat2 = degToRad(lat2);
    lon2 = degToRad(lon2);

    double dLon = lon2 - lon1;
    // Normalize dLon to be within -PI to PI range
    dLon = fmod((dLon + 3 * M_PI), (2 * M_PI)) - M_PI;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x) * 180.0 / M_PI;
    bearing = fmod((bearing + 360.0), 360.0);  // Adjust bearing to be within [0, 360] range
    return bearing;
}

#endif
