#include "include/mpu6050.h"
#include "config.h"
#include "kalman_filter.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";

// MPU6050 Register Addresses
#define MPU_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_CONFIG 0x1A
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_MOT_THR 0x1F     // Motion threshold register
#define MPU6050_MOT_DUR 0x20     // Motion duration register
#define MPU6050_INT_PIN_CFG 0x37 // Interrupt pin config
#define MPU6050_INT_ENABLE 0x38  // Interrupt enable register
#define MPU6050_INT_STATUS 0x3A  // Interrupt status register

#define MPU6050_ACCEL_FS_SEL_8G 0x10
#define MPU6050_BAND_21_HZ 0x04

// Kalman filter handle
static mpu_kalman_filter_handle_t kf_handle;

// Interrupt flag
static volatile bool mpu_interrupt_flag = false;

// Helper function to write to a MPU6050 register
static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[] = {reg, value};
    return i2c_master_write_to_device(I2C_NUM_0, MPU_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

// Helper function to read MPU6050 registers
static esp_err_t mpu6050_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_NUM_0, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

esp_err_t mpu6050_init(void)
{
    // The I2C driver should be installed before calling this
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for sensor to power up

    // Wake up MPU
    esp_err_t err = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check if device is responsive
    uint8_t check;
    err = mpu6050_read_regs(0x75, &check, 1); // WHO_AM_I register
    if (err != ESP_OK || check != MPU_ADDR)
    {
        ESP_LOGE(TAG, "MPU6050 not found at address 0x%02X. Read: 0x%02X", MPU_ADDR, check);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU6050 found at address 0x%02X", MPU_ADDR);

    // Set accelerometer range to +/- 8G
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL_8G);
    // Set digital low pass filter to 21 Hz
    mpu6050_write_reg(MPU6050_CONFIG, MPU6050_BAND_21_HZ);

    // Create Kalman filter instance
    kf_handle = mpu_kalman_filter_create(0.5, 1.0, 0.05);
    if (!kf_handle)
    {
        ESP_LOGE(TAG, "Failed to create MPU Kalman filter");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6050 Initialized OK. Warming up Kalman filter...");
    for (int i = 0; i < 20; i++)
    {
        mpu6050_calc_deviation(); // Run a few times to stabilize the filter
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Kalman filter ready");

    return ESP_OK;
}

void mpu6050_put_to_sleep(void)
{
    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x40); // Set SLEEP bit
    ESP_LOGI(TAG, "MPU6050 sleep command sent");
}

void mpu6050_wake_from_sleep(void)
{
    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00); // Clear SLEEP bit
    vTaskDelay(pdMS_TO_TICKS(50));               // Wait for sensor to stabilize

    // Warm up Kalman filter again after waking up
    for (int i = 0; i < 10; i++)
    {
        mpu6050_calc_deviation();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "MPU6050 wake command sent, Kalman filter warmed up");
}

float mpu6050_calc_deviation(void)
{
    uint8_t data[6];
    esp_err_t err = mpu6050_read_regs(MPU6050_ACCEL_XOUT_H, data, 6);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read accel data");
        return 0.0f;
    }

    // Convert raw data to m/s^2
    // Raw value is signed 16-bit
    int16_t ax_raw = (int16_t)((data[0] << 8) | data[1]);
    int16_t ay_raw = (int16_t)((data[2] << 8) | data[3]);
    int16_t az_raw = (int16_t)((data[4] << 8) | data[5]);

    // Sensitivity for +/- 8g range is 4096 LSB/g
    const float G_TO_MS2 = 9.80665f;
    const float LSB_PER_G = 4096.0f;
    float ax = (ax_raw / LSB_PER_G) * G_TO_MS2;
    float ay = (ay_raw / LSB_PER_G) * G_TO_MS2;
    float az = (az_raw / LSB_PER_G) * G_TO_MS2;

    // Apply Kalman filter
    float fx, fy, fz;
    mpu_kalman_filter_update(kf_handle, ax, ay, az, &fx, &fy, &fz);

    // Calculate deviation
    float total_acc = sqrt(fx * fx + fy * fy + fz * fz);
    float deviation = fabs(total_acc - G_TO_MS2);

    // Apply deadband
    if (deviation < 1.0)
    {
        deviation = 0.0f;
    }
    return deviation;
}

esp_err_t mpu6050_config_motion_interrupt(uint8_t threshold, uint8_t duration)
{
    esp_err_t ret;

    // Wake up if sleeping
    ret = mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set accelerometer to ±2g range for motion detection
    ret = mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK)
        return ret;

    // Configure DLPF (Digital Low Pass Filter) off
    ret = mpu6050_write_reg(MPU6050_CONFIG, 0x00);
    if (ret != ESP_OK)
        return ret;

    // Set motion detection threshold (0-255, 1 LSB = 2mg at ±2g)
    ret = mpu6050_write_reg(MPU6050_MOT_THR, threshold);
    if (ret != ESP_OK)
        return ret;

    // Set motion detection duration (0-255, 1 LSB = 1ms)
    ret = mpu6050_write_reg(MPU6050_MOT_DUR, duration);
    if (ret != ESP_OK)
        return ret;

    // Configure interrupt pin: active high, push-pull
    ret = mpu6050_write_reg(MPU6050_INT_PIN_CFG, 0x20);
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(TAG, "Motion interrupt configured: threshold=%d, duration=%d", threshold, duration);
    return ESP_OK;
}

esp_err_t mpu6050_enable_motion_interrupt(void)
{
    // Enable motion detection interrupt (bit 6)
    esp_err_t ret = mpu6050_write_reg(MPU6050_INT_ENABLE, 0x40);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Motion interrupt enabled");
    }
    return ret;
}

esp_err_t mpu6050_disable_motion_interrupt(void)
{
    // Disable all interrupts
    esp_err_t ret = mpu6050_write_reg(MPU6050_INT_ENABLE, 0x00);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Motion interrupt disabled");
    }
    return ret;
}

uint8_t mpu6050_get_interrupt_status(void)
{
    uint8_t status = 0;
    mpu6050_read_regs(MPU6050_INT_STATUS, &status, 1);
    return status;
}

void mpu6050_isr_handler(void)
{
    mpu_interrupt_flag = true;
}

bool mpu6050_get_interrupt_flag(void)
{
    return mpu_interrupt_flag;
}

void mpu6050_clear_interrupt_flag(void)
{
    mpu_interrupt_flag = false;
}