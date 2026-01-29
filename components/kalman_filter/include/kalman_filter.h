#ifndef KALMAN_FILTER_C_H
#define KALMAN_FILTER_C_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer types to hide C++ implementation details
typedef struct GPSKalmanFilter_t* gps_kalman_filter_handle_t;
typedef struct MPUKalmanFilter_t* mpu_kalman_filter_handle_t;

// Functions for GPSKalmanFilter
gps_kalman_filter_handle_t gps_kalman_filter_create(float measurement_uncertainty, float estimation_uncertainty, float process_noise);
void gps_kalman_filter_delete(gps_kalman_filter_handle_t handle);
void gps_kalman_filter_reset(gps_kalman_filter_handle_t handle, float initial_lat, float initial_lng);
void gps_kalman_filter_update(gps_kalman_filter_handle_t handle, float measured_lat, float measured_lng, float* filtered_lat, float* filtered_lng);

// Functions for MPUKalmanFilter
mpu_kalman_filter_handle_t mpu_kalman_filter_create(float measurement_uncertainty, float estimation_uncertainty, float process_noise);
void mpu_kalman_filter_delete(mpu_kalman_filter_handle_t handle);
void mpu_kalman_filter_reset(mpu_kalman_filter_handle_t handle);
void mpu_kalman_filter_update(mpu_kalman_filter_handle_t handle, float measured_x, float measured_y, float measured_z, float* filtered_x, float* filtered_y, float* filtered_z);

#ifdef __cplusplus
}
#endif

#endif // KALMAN_FILTER_C_H
