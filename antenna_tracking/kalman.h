#ifndef KALMAN_H
#define KALMAN_H

#include <cmath>
#include <algorithm>

class KalmanFilter {
public:
    KalmanFilter();
    void init(float initial_estimate, float initial_error);
    float update(float measurement);

private:
    float estimate;
    float error;
    float measurement_error;
    float process_noise;
    float kalman_gain;
    float previous_measurement;
    bool is_first_measurement;
};

KalmanFilter::KalmanFilter() : 
    estimate(0), 
    error(1), 
    measurement_error(1), 
    process_noise(0.1), 
    kalman_gain(0),
    previous_measurement(0),
    is_first_measurement(true) {}

void KalmanFilter::init(float initial_estimate, float initial_error) {
    estimate = initial_estimate;
    error = initial_error;
}

float KalmanFilter::update(float measurement) {
    if (is_first_measurement) {
        previous_measurement = measurement;
        is_first_measurement = false;
    }

    // Adaptive tuning based on the difference between current and previous measurement
    float measurement_diff = std::fabs(measurement - previous_measurement);
    measurement_error = std::max(0.01f, measurement_diff * 0.5f);  // Adjust this factor based on application
    process_noise = std::max(0.01f, measurement_diff * 0.1f);     // Adjust this factor based on application

    kalman_gain = error / (error + measurement_error);
    estimate = estimate + kalman_gain * (measurement - estimate);
    error = (1 - kalman_gain) * error + std::fabs(estimate - measurement) * process_noise;

    previous_measurement = measurement;
    return estimate;
}

#endif // KALMAN_H
