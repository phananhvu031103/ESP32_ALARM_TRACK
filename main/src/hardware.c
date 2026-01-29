#include "hardware.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "HARDWARE";

adc_oneshot_unit_handle_t g_adc_handle = NULL;
static bool i2c_initialized = false;
static bool uart_initialized = false;
static bool gpio_initialized = false;
static bool adc_initialized = false;

esp_err_t hardware_init_i2c(void)
{
    if (i2c_initialized)
    {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_initialized = true;
    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

esp_err_t hardware_init_uarts(void)
{
    if (uart_initialized)
    {
        ESP_LOGW(TAG, "UARTs already initialized");
        return ESP_OK;
    }

    // Configure GPS UART
    uart_config_t gps_uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(GPS_UART_NUM, &gps_uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPS UART config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPS UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(GPS_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPS UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SIM UART
    uart_config_t sim_uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_param_config(SIM_UART_NUM, &sim_uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SIM UART config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(SIM_UART_NUM, SIM_TX_PIN, SIM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SIM UART set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(SIM_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SIM UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uart_initialized = true;
    ESP_LOGI(TAG, "UARTs initialized successfully");
    return ESP_OK;
}

esp_err_t hardware_init_gpio(void)
{
    if (gpio_initialized)
    {
        ESP_LOGW(TAG, "GPIO already initialized");
        return ESP_OK;
    }

    // Configure output pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUZZER_PIN) | (1ULL << LED_ALERT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Output GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set initial state
    gpio_set_level(BUZZER_PIN, 0);
    gpio_set_level(LED_ALERT_PIN, 0);

    // Configure input pins (buttons)
    io_conf.pin_bit_mask = (1ULL << MODE_BUTTON_PIN) | (1ULL << RESET_BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Input GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure MPU interrupt pin
    io_conf.pin_bit_mask = (1ULL << MPU_INT_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU INT GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_initialized = true;
    ESP_LOGI(TAG, "GPIO initialized successfully");
    return ESP_OK;
}

esp_err_t hardware_init_adc(void)
{
    if (adc_initialized)
    {
        ESP_LOGW(TAG, "ADC already initialized");
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &g_adc_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    ret = adc_oneshot_config_channel(g_adc_handle, ADC_CHANNEL_6, &config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(g_adc_handle);
        return ret;
    }

    adc_initialized = true;
    ESP_LOGI(TAG, "ADC initialized successfully");
    return ESP_OK;
}

esp_err_t hardware_init(void)
{
    esp_err_t ret;

    ret = hardware_init_gpio();
    if (ret != ESP_OK)
        return ret;

    ret = hardware_init_i2c();
    if (ret != ESP_OK)
        return ret;

    ret = hardware_init_uarts();
    if (ret != ESP_OK)
        return ret;

    ret = hardware_init_adc();
    if (ret != ESP_OK)
        return ret;

    ESP_LOGI(TAG, "All hardware initialized successfully");
    return ESP_OK;
}
