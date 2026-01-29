#include "kalman_filter.hpp"
#include "include/kalman_filter.h"

// Bridge C++ classes to C-style handles
// The C code only sees the typedef'd pointers, not the C++ class definitions.
struct GPSKalmanFilter_t : public GPSKalmanFilter {
    GPSKalmanFilter_t(float mu, float eu, float pn) : GPSKalmanFilter(mu, eu, pn) {}
};

struct MPUKalmanFilter_t : public MPUKalmanFilter {
    MPUKalmanFilter_t(float mu, float eu, float pn) : MPUKalmanFilter(mu, eu, pn) {}
};

extern "C" {

// GPS Kalman Filter C-wrapper implementation
gps_kalman_filter_handle_t gps_kalman_filter_create(float measurement_uncertainty, float estimation_uncertainty, float process_noise) {
    return new GPSKalmanFilter_t(measurement_uncertainty, estimation_uncertainty, process_noise);
}

void gps_kalman_filter_delete(gps_kalman_filter_handle_t handle) {
    delete handle;
}

void gps_kalman_filter_reset(gps_kalman_filter_handle_t handle, float initial_lat, float initial_lng) {
    if (handle) {
        handle->reset(initial_lat, initial_lng);
    }
}

void gps_kalman_filter_update(gps_kalman_filter_handle_t handle, float measured_lat, float measured_lng, float* filtered_lat, float* filtered_lng) {
    if (handle) {
        handle->updatePosition(measured_lat, measured_lng, filtered_lat, filtered_lng);
    }
}

// MPU Kalman Filter C-wrapper implementation
mpu_kalman_filter_handle_t mpu_kalman_filter_create(float measurement_uncertainty, float estimation_uncertainty, float process_noise) {
    return new MPUKalmanFilter_t(measurement_uncertainty, estimation_uncertainty, process_noise);
}

void mpu_kalman_filter_delete(mpu_kalman_filter_handle_t handle) {
    delete handle;
}

void mpu_kalman_filter_reset(mpu_kalman_filter_handle_t handle) {
    if (handle) {
        handle->reset();
    }
}

void mpu_kalman_filter_update(mpu_kalman_filter_handle_t handle, float measured_x, float measured_y, float measured_z, float* filtered_x, float* filtered_y, float* filtered_z) {
    if (handle) {
        handle->updateAcceleration(measured_x, measured_y, measured_z, filtered_x, filtered_y, filtered_z);
    }
}

} // extern "C"
