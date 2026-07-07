#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_sntp.h"
#include "time.h"
#include <sys/time.h>
#include "esp_sleep.h"
#include "env_config.h"
#include "esp_mac.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdint.h>
#include "esp_spiffs.h"
#include <sys/stat.h>
#include "cJSON.h"
#include "panel_config.h"
#include "mdns.h"
#include "esp_netif.h"
#include "esp_https_ota.h"
#include "esp_adc/adc_oneshot.h"

adc_oneshot_unit_handle_t adc_handle;

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

volatile bool ota_in_progress = false;

panel_config_t g_panel_config;

static const char *OTA_TAG = "OTA";
static TaskHandle_t ota_task_handle = NULL;

static const char *TAG_HTTP = "HTTP_TIME";
static bool sntp_initialized = false;
static char macID[13];
char panelName[50];

void obtain_time(void);
bool is_daytime(void);
void send_eeprom_data(void);
esp_err_t send_to_influx_batch(const char *payload);

extern const uint8_t InfluxRootCA_pem_start[] asm("_binary_InfluxRootCA_pem_start");
extern const uint8_t InfluxRootCA_pem_end[]   asm("_binary_InfluxRootCA_pem_end");

// Wifi configurations included in env_config.h

// I2C (ADC)
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
//#define MCP3426_ADDR                0x6E  // I2C address of MCP3426 blue board
#define MCP3426_ADDR                0x6B  // I2C address of MCP3426 green board
#define TEMP_SENSOR_ENABLE_GPIO 25
//define GY21_ADDR
#define GY21_ADDR                   0x40
#define GY21_CMD_TEMP_NO_HOLD       0xF3
#define GY21_CMD_HUM_NO_HOLD        0xF5

// SPI (MAX31865)
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15


// MAX31865 TEMPERATURE
#define MAX31865_REG_CONFIG  0x00
#define MAX31865_REG_RTD_MSB 0x01
#define MAX31865_CONFIG_BIAS_ON   0x80
#define MAX31865_CONFIG_AUTO_CONV 0x40 
#define MAX31865_CONFIG_4WIRE     0x00
static const float A = 3.9083e-3;
static const float B = -5.775e-7;

#define SENSOR_INVALID_VALUE   0

#define BUZZER_PIN 2
#define RESET_HOLD_GPIO 32

//ota status variables
static const char *ota_status = "idle";
static esp_reset_reason_t last_reset_reason;


// GLOBALS 
static const char *TAGMQTT = "ESP32_ADC_TEMP_MQTT";
// static esp_mqtt_client_handle_t mqtt_client;
static char topic[64];
static spi_device_handle_t spi; // MAX31865 SPI handle
static SemaphoreHandle_t i2c_semaphore = NULL;


//  time variables for sending data
#define SUNRISE_HOUR 6
#define SUNSET_HOUR 18

#define EEPROM_SIZE 1024      // 1 KB for example
#define SAMPLE_SIZE sizeof(sensor_sample_t)
#define MAX_SAMPLES (EEPROM_SIZE / SAMPLE_SIZE)

#define CURRENT_FIRMWARE_VERSION "1.0.0"
#define OTA_TOTAL_TIMEOUT_MS (5 * 60 * 1000) //OTA is given 5 minutes to complete or else revert back

//checks OTA version before updating
#define VERSION_URL "https://raw.githubusercontent.com/uni-stuttgart-ipv/rooftop_sensors/refs/heads/dev/version.json"
//#define VERSION_URL "https://raw.githubusercontent.com/Neha-kb/rooftop_sensors/refs/heads/dev/version.json"


typedef struct {
    float voltage;
    float current;
    float power;
    float temperature;
    float ambient_temperature;  // GY-21 air temperature
    float humidity;             // GY-21 humidity
    float irradiance;   
    time_t timestamp;
} sensor_sample_t;

static QueueHandle_t sensor_queue;

sensor_sample_t eeprom_buffer[MAX_SAMPLES];
int eeprom_index = 0;  // next free slot

void send_eeprom_task(void *pvParameters) {
    send_eeprom_data();
    vTaskDelete(NULL);
}
 
//checking log size to decide if it needs to be cleared
bool is_log_too_large(size_t max_size)
{
    struct stat st;
    if (stat("/spiffs/logs.txt", &st) == 0) {
        return st.st_size > max_size;
    }
    return false; // file doesn't exist yet
}

void clear_logs_spiffs()
{
    remove("/spiffs/logs.txt");
}

void reset_hold_high(void)
{
    gpio_reset_pin(RESET_HOLD_GPIO);
    gpio_set_direction(RESET_HOLD_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_HOLD_GPIO, 1);
    ESP_LOGI(TAGMQTT, "GPIO32 reset hold set HIGH");
}

void init_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "spiffs",
        .max_files = 5,       // maximum open files at a time
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE("SPIFFS", "Failed to mount SPIFFS");
    } else {
        size_t total = 0, used = 0;
        esp_spiffs_info("spiffs", &total, &used);
        ESP_LOGI("SPIFFS", "Total: %d, Used: %d", total, used);
    }
}

//write logs to file
void save_log_spiffs(const char *message)
{
    struct stat st;
    if (stat("/spiffs/logs.txt", &st) == 0) {
        if (st.st_size > 5000) {
            ESP_LOGW("SPIFFS", "Log file too large, clearing...");
            remove("/spiffs/logs.txt");  // delete old logs
        }
    }

    //Open file
    FILE *f = fopen("/spiffs/logs.txt", "a");
    if (f == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open log file");
        return;
    }

    //Write log
    fprintf(f, "%s\n", message);

    fclose(f);
}

//read logs
void print_logs_spiffs() {
    FILE *f = fopen("/spiffs/logs.txt", "r");
    if (f == NULL) {
        ESP_LOGE("SPIFFS", "No logs found");
        return;
    }

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        printf("%s", line);  // print to UART / monitor
    }

    fclose(f);
}


void buzzer_init()
{
    gpio_reset_pin(BUZZER_PIN);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_PIN, 0);
}

// Play a single note of given frequency (Hz) and duration (ms)
void play_note(uint32_t freq, uint32_t duration_ms)
{
    if (freq == 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms)); // silence
        return;
    }

    uint32_t period_us = 1000000 / freq;  // microseconds per cycle
    uint32_t half_period = period_us / 2;
    uint32_t start_time = esp_timer_get_time();
    uint32_t end_time = start_time + (duration_ms * 1000);

    while (esp_timer_get_time() < end_time) {
        gpio_set_level(BUZZER_PIN, 1);
        uint32_t t = esp_timer_get_time();
        while ((esp_timer_get_time() - t) < half_period) {} // busy wait

        gpio_set_level(BUZZER_PIN, 0);
        t = esp_timer_get_time();
        while ((esp_timer_get_time() - t) < half_period) {}
    }
}

void play_wifi_connected()
{
    play_note(523, 100);  // C5
    play_note(659, 100);  // E5
    play_note(784, 150);  // G5
    play_note(1047, 200); // C6
}

void play_voltage_sensed()
{
    play_note(1000, 40);
}

void play_data_sent()
{
    play_note(784, 80);
    play_note(659, 80);
}

//  HELPER: MAC ADDRESS 
void get_mac_address(char *mac_str, size_t len) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_str, len, "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}


static const char *TAG = "MCP3426";

esp_err_t perform_ota_update(const char *url)
{
    ESP_LOGI(OTA_TAG, "Starting OTA: %s", url);

    esp_http_client_config_t http_config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,
        .buffer_size = 8192,
        .buffer_size_tx = 1024,
        .keep_alive_enable = false,
        .disable_auto_redirect = false,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t ret = esp_https_ota_begin(&ota_config, &ota_handle);

    if (ret != ESP_OK) {
        ESP_LOGE(OTA_TAG, "OTA begin failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int64_t start_time_us = esp_timer_get_time();

    while (1) {
        ret = esp_https_ota_perform(ota_handle);

        if (ret != ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
            break;
        }

        int64_t elapsed_ms = (esp_timer_get_time() - start_time_us) / 1000;

        if (elapsed_ms > OTA_TOTAL_TIMEOUT_MS) {
            ESP_LOGE(OTA_TAG, "OTA timeout after %lld ms, aborting", elapsed_ms);
            esp_https_ota_abort(ota_handle);
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (ret != ESP_OK) {
        ESP_LOGE(OTA_TAG, "OTA perform failed: %s", esp_err_to_name(ret));
        esp_https_ota_abort(ota_handle);
        return ret;
    }

    if (!esp_https_ota_is_complete_data_received(ota_handle)) {
        ESP_LOGE(OTA_TAG, "OTA image not fully received");
        esp_https_ota_abort(ota_handle);
        return ESP_FAIL;
    }

    ret = esp_https_ota_finish(ota_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(OTA_TAG, "OTA success, rebooting...");
        esp_restart();
    }

    ESP_LOGE(OTA_TAG, "OTA finish failed: %s", esp_err_to_name(ret));
    return ret;
}

static esp_err_t fetch_version_json(char *out_buf, size_t max_len)
{
    char version_url[300];

    snprintf(version_url, sizeof(version_url),
             "%s?nocache=%lld",
             VERSION_URL,
             esp_timer_get_time());

    ESP_LOGI("OTA", "Fetching version URL: %s", version_url);

    esp_http_client_config_t config = {
        .url = version_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 5000,
        .buffer_size = 4096,
        .keep_alive_enable = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE("OTA", "HTTP init failed");
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Cache-Control", "no-cache");
    esp_http_client_set_header(client, "Pragma", "no-cache");

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "HTTP open failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int content_length = esp_http_client_fetch_headers(client);
    ESP_LOGI("OTA", "Content length: %d", content_length);

    int read_len = esp_http_client_read_response(client, out_buf, max_len - 1);

    if (read_len <= 0) {
        ESP_LOGE("OTA", "Read failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    out_buf[read_len] = '\0';

    ESP_LOGI("OTA", "RAW JSON:\n%s", out_buf);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    return ESP_OK;
}

void check_for_update(void)
{
    char json[512];

    ESP_LOGI("OTA", "Fetching version JSON...");

    if (fetch_version_json(json, sizeof(json)) != ESP_OK) {
        ESP_LOGE("OTA", "Failed to fetch version JSON");
        return;
    }

    ESP_LOGI("OTA", "RAW JSON:\n%s", json);

    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE("OTA", "JSON parse failed");
        return;
    }

    cJSON *version_item = cJSON_GetObjectItem(root, "version");
    cJSON *url_item = cJSON_GetObjectItem(root, "firmware_url");

    if (!cJSON_IsString(version_item) || !cJSON_IsString(url_item)) {
        ESP_LOGE("OTA", "Invalid JSON");
        cJSON_Delete(root);
        return;
    }

    const char *latest_version = version_item->valuestring;
    const char *firmware_url = url_item->valuestring;

    ESP_LOGI("OTA", "Latest version: %s", latest_version);

    if (strcmp(latest_version, CURRENT_FIRMWARE_VERSION) != 0) {
    ESP_LOGI("OTA", "New firmware found!");

    ota_in_progress = true;
    vTaskDelay(pdMS_TO_TICKS(1500));

    esp_err_t ota_ret = perform_ota_update(firmware_url);

    ota_in_progress = false;

    if (ota_ret != ESP_OK) {
        ESP_LOGE("OTA", "OTA failed or timed out, resuming normal firmware");
        cJSON_Delete(root);
        return;
    }
    } else {
    ESP_LOGI("OTA", "Already up to date");
    }

    cJSON_Delete(root);
}

void ota_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for WiFi stable
    while (1) {
        ESP_LOGI("OTA", "Periodic OTA check...");
        check_for_update();

        vTaskDelay(pdMS_TO_TICKS(2 * 60 * 1000)); // every 2 minutes checking new firmware
    }
}


// I²C initialization
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

void irradiance_adc_init()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    ESP_LOGI("ADC", "Handle after init = %p", adc_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle,
        ADC_CHANNEL_6,   // GPIO34
        &config
    ));
}

//Start a one-shot 12-bit conversion
static esp_err_t mcp3426_start_conversion(uint8_t channel)
{
    uint8_t config = 0b10000000; // start, CH1, one-shot, 12-bit, gain=1
    if (channel == 2) config |= 0b00100000;  // select CH2
    return i2c_master_write_to_device(I2C_MASTER_NUM, MCP3426_ADDR,
                                      &config, 1, pdMS_TO_TICKS(50));
}

// Read 12-bit result
static esp_err_t mcp3426_read_12bit(int16_t *result)
{
    uint8_t data[3];
    while (1)
    {
        esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, MCP3426_ADDR,
                                                    data, sizeof(data), pdMS_TO_TICKS(50));
        if (err != ESP_OK) return err;

        uint8_t cfg = data[2];
        if ((cfg & 0x80) == 0) {  // RDY=0 → data ready
            int16_t raw = (int16_t)((data[0] << 8) | data[1]);
            *result = raw;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// Wrapper to get ADC reading for a channel
int getADC(uint8_t channel)
{
    int16_t raw;
    if (mcp3426_start_conversion(channel) != ESP_OK) {
        ESP_LOGE(TAGMQTT, "Failed to start conversion on channel %d", channel);
        return 0;
    }
    if (mcp3426_read_12bit(&raw) != ESP_OK) {
        ESP_LOGE(TAGMQTT, "Failed to read ADC on channel %d", channel);
        return 0;
    }
    return raw;
}


float getVoltage()
{
    int adcReading = getADC(1);
    float voltage = ((float)adcReading / 2047.0f) * 2.048f; 
    voltage *= (118.0f + 4.02f) / 4.02f;                     // voltage divider
    //voltage *= 1.95; 
    play_voltage_sensed();
    save_log_spiffs("Voltage measured");
    return voltage;
}

float getCurrent()
{
    int adcReading = getADC(2);
    float voltage = ((float)adcReading / 2047.0f) * 2.048f; 
    float current = (voltage * 1.62f) - 0.33f; // apply sensor scaling factor
    //current = (current/0.264f)*1.975;
    current = (current/0.264f);
    return (current < 0.0f) ? 0.0f : current;
}

float getIrradiance()
{
    int raw;

    adc_oneshot_read(
        adc_handle,
        ADC_CHANNEL_6,
        &raw
    );

    float voltage = (raw / 4095.0f) * 3.3f;

    return voltage * 1000.0f;
}

void temp_sensor_power_on(void)
{
    gpio_reset_pin(TEMP_SENSOR_ENABLE_GPIO);
    gpio_set_direction(TEMP_SENSOR_ENABLE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(TEMP_SENSOR_ENABLE_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
}

//  MAX31865 INIT
void max31865_init(void) {
    temp_sensor_power_on();
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 1,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));
    uint8_t tx_data[2] = { (MAX31865_REG_CONFIG | 0x80),
                           MAX31865_CONFIG_BIAS_ON | MAX31865_CONFIG_AUTO_CONV | MAX31865_CONFIG_4WIRE };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx_data };
    ret = spi_device_polling_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAGMQTT, "MAX31865 initialized");
}

uint16_t max31865_read_rtd(void) {
    uint8_t tx_data[3] = { MAX31865_REG_RTD_MSB, 0x00, 0x00 };
    uint8_t rx_data[3] = {0};
    spi_transaction_t t = {0};
    t.length = 24; // bits
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    t.flags = 0;
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAGMQTT, "SPI transmit failed: %d", ret);
        return 0;
    }
    uint16_t rtd = ((rx_data[1] << 8) | rx_data[2]) >> 1;
    return rtd;
}

float getTemperature() {
    uint16_t rtd = max31865_read_rtd();
    const float Rref = 430.0f;
    const float R0 = 100.0f;
    float Rt = ((float)rtd * Rref) / 32768.0f;
    float temp = (-A + sqrtf(A*A - 4*B*(1 - Rt/R0))) / (2*B);
    return temp;
}

static esp_err_t gy21_read_raw(uint8_t command, uint16_t *raw_value, uint32_t delay_ms)
{
    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        GY21_ADDR,
        &command,
        1,
        pdMS_TO_TICKS(100)
    );

    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    uint8_t data[3] = {0};

    err = i2c_master_read_from_device(
        I2C_MASTER_NUM,
        GY21_ADDR,
        data,
        3,
        pdMS_TO_TICKS(100)
    );

    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    raw &= 0xFFFC;

    *raw_value = raw;
    return ESP_OK;
}

float getAmbientTemperatureGY21(void)
{
    uint16_t raw = 0;

    esp_err_t err = gy21_read_raw(GY21_CMD_TEMP_NO_HOLD, &raw, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAGMQTT, "Failed to read GY-21 temperature: %s", esp_err_to_name(err));
        return SENSOR_INVALID_VALUE;
    }

    return -46.85f + (175.72f * (float)raw / 65536.0f);
}

float getHumidityGY21(void)
{
    uint16_t raw = 0;

    esp_err_t err = gy21_read_raw(GY21_CMD_HUM_NO_HOLD, &raw, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAGMQTT, "Failed to read GY-21 humidity: %s", esp_err_to_name(err));
        return SENSOR_INVALID_VALUE;
    }

    float humidity = -6.0f + (125.0f * (float)raw / 65536.0f);

    if (humidity < 0.0f) humidity = 0.0f;
    if (humidity > 100.0f) humidity = 100.0f;

    return humidity;
}

//bool mqtt_ready = false;

// Wi-Fi event handler for reconnection
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAGMQTT, "Wi-Fi disconnected. Reconnecting...");
        save_log_spiffs("Wi-Fi disconnected. Reconnecting...");
        esp_wifi_connect();  // attempt reconnect
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAGMQTT, "Wi-Fi associated with AP, waiting for IP...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAGMQTT, "Wi-Fi got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        save_log_spiffs("WiFi connected");

        play_wifi_connected();

        //We disable power save mode to ensure stable connection
        esp_wifi_set_ps(WIFI_PS_NONE);

        //Initialising SNTP
        if (!sntp_initialized) {
        obtain_time();
        sntp_initialized = true;
    }
    
    //creating a small task to do eeprom data transfer
    xTaskCreate(send_eeprom_task, "send_eeprom_task", 8192, NULL, 5, NULL);
    
    if (ota_task_handle == NULL) {
        xTaskCreate(ota_task, "ota_task", 12288, NULL, 5, &ota_task_handle);
    }
    }
}

//  WIFI INIT 
void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    
    // Register event handler here
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL
    ));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL
    ));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAGMQTT, "Connecting to WiFi %s...", WIFI_SSID);
    vTaskDelay(pdMS_TO_TICKS(5000)); // initial delay to allow connection
}


void obtain_time(void) {
    ESP_LOGI(TAGMQTT, "Initializing SNTP...");
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "time3.uni-stuttgart.de");
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    esp_sntp_init();


    // Wait for time synchronization
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAGMQTT, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year >= (2016 - 1900)) {
        ESP_LOGI(TAGMQTT, "Time synchronized successfully.");
        char strftime_buf[64];
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAGMQTT, "Current local time: %s", strftime_buf);
        char log_msg[128];
        snprintf(log_msg, sizeof(log_msg), "Time: %s", strftime_buf);
        save_log_spiffs(log_msg);
    } 
    else {
        ESP_LOGW(TAGMQTT, "Failed to synchronize time.");
    }
}



bool is_daytime(void) {
    // time_t now;
    // struct tm timeinfo;
    // time(&now);
    // localtime_r(&now, &timeinfo);

    // int hour = timeinfo.tm_hour;
    // return (hour >= SUNRISE_HOUR && hour < SUNSET_HOUR);

    //ESP_LOGI(TAGMQTT, "Mock daytime for testing");
    return true;

//     ESP_LOGI(TAGMQTT, "Mock night for testing");
//     return false;
}



void sleep_until_sunrise(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    int hours_to_sunrise = (24 + SUNRISE_HOUR - timeinfo.tm_hour) % 24;
    int seconds_to_sunrise = hours_to_sunrise * 3600;

    ESP_LOGI(TAGMQTT, "Sleeping for %d hours until sunrise...", hours_to_sunrise);
    esp_sleep_enable_timer_wakeup((uint64_t)seconds_to_sunrise * 1000000ULL);
    esp_deep_sleep_start();
}


// CHECK WIFI CONNECTIVITY 
bool wifi_connected() {
    wifi_ap_record_t ap_info;
    return (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK);
}


void send_to_influx(sensor_sample_t sample) {
    if (ota_in_progress) {
        ESP_LOGI(TAGMQTT, "OTA in progress, skipping InfluxDB upload");
        return;
    }

    char line1[256] = {0};
    char line2[256] = {0};
    char payload[512] = {0};
    time_t ts = sample.timestamp;

    switch (g_panel_config.board_type)
    {
        case BOARD_NORMAL:
        snprintf(line1, sizeof(line1),
            "panel_parameters,mcu=%s,string=%s,panel_sn=%s,location=%s voltage=%.2f,current=%.2f,power=%.2f,temperature=%.2f %lld",
            macID, g_panel_config.String, g_panel_config.panel_sn, g_panel_config.location, sample.voltage, sample.current, sample.power, sample.temperature,ts
        );
        break;
    case BOARD_GY21:
        snprintf(line1, sizeof(line1),
            "panel_parameters,mcu=%s,string=%s,panel_sn=%s,location=%s voltage=%.2f,current=%.2f,power=%.2f,temperature=%.2f %lld",
            macID, g_panel_config.String, g_panel_config.panel_sn, g_panel_config.location, sample.voltage, sample.current, sample.power, sample.temperature,ts
        );
        snprintf(line2, sizeof(line2),
            "temperature_humidity,mcu=%s ambient_temperature=%.2f,humidity=%.2f %lld",
            macID,sample.ambient_temperature,sample.humidity,ts
        );
        break;
    case BOARD_IRRADIANCE:
        snprintf(line1, sizeof(line1),
            "panel_parameters,mcu=%s,string=%s,panel_sn=%s,location=%s voltage=%.2f,current=%.2f,power=%.2f,temperature=%.2f %lld",
            macID, g_panel_config.String, g_panel_config.panel_sn, g_panel_config.location, sample.voltage, sample.current, sample.power, sample.temperature,ts
        );
        snprintf(line2, sizeof(line2),
            "irradiance,mcu=%s irradance=%.2f %lld",
            macID,sample.irradiance,ts
        );
        break;
    default:
        ESP_LOGE(TAGMQTT, "Unknown board type: %d", g_panel_config.board_type);
        return;
    }
    
    if (line2[0] != '\0') {
        snprintf(payload, sizeof(payload), "%s\n%s", line1, line2);
    } else {
        snprintf(payload, sizeof(payload), "%s", line1);
    }

    printf("INFLUX PAYLOAD:\n%s\n", payload);

    char url[256];
    snprintf(url, sizeof(url),
             "%s/api/v2/write?org=%s&bucket=%s&precision=s",
             INFLUX_URL, INFLUX_ORG, INFLUX_BUCKET);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    char auth_header[256];
    snprintf(auth_header, sizeof(auth_header), "Token %s", INFLUX_TOKEN);

    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8");

    esp_http_client_set_post_field(client, payload, strlen(payload));

    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);

        if (status == 204) {
            ESP_LOGI(TAGMQTT, "InfluxDB write OK");
            save_log_spiffs("Data sent to InfluxDB");
            play_data_sent();
        } else {
            ESP_LOGE(TAGMQTT, "InfluxDB HTTP error: %d", status);
        }
    } else {
        ESP_LOGE(TAGMQTT, "InfluxDB POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

bool is_eeprom_too_large(size_t max_size)
{
    struct stat st;

    if (stat("/spiffs/eeprom_data.bin", &st) == 0) {
        return st.st_size > max_size;
    }

    return false;
}


void save_eeprom_data(sensor_sample_t *sample)
{
    // Check if file is too large BEFORE writing
    if (is_eeprom_too_large(50 * 1024)) {   // 50 KB limit
        ESP_LOGW("SPIFFS", "EEPROM file too large, clearing...");
        remove("/spiffs/eeprom_data.bin");
    }
    FILE *f = fopen("/spiffs/eeprom_data.bin", "ab");
    if (!f) {
        ESP_LOGE("SPIFFS", "Failed to open EEPROM file");
        return;
    }

    fwrite(sample, sizeof(sensor_sample_t), 1, f);
    fclose(f);

    ESP_LOGI("SPIFFS", "EEPROM sample saved");
}

//to print data stored in eeprom
void print_eeprom_data()
{
    FILE *f = fopen("/spiffs/eeprom_data.bin", "rb");
    if (!f) {
        ESP_LOGW(TAGMQTT, "No EEPROM data found");
        return;
    }

    sensor_sample_t sample;
    int i = 0;

    ESP_LOGI(TAGMQTT, "----- EEPROM DATA START -----");

    while (fread(&sample, sizeof(sensor_sample_t), 1, f) == 1) {
        ESP_LOGI(TAGMQTT,
                 "EEPROM[%d]: V=%.2f V | I=%.2f A | P=%.2f W | T=%.2f °C | TS=%lld",
                 i,
                 sample.voltage,
                 sample.current,
                 sample.power,
                 sample.temperature,
                 (long long)sample.timestamp);
        i++;
    }

    ESP_LOGI(TAGMQTT, "----- EEPROM DATA END (%d samples) -----", i);

    fclose(f);
}

void send_eeprom_data()
{
    FILE *f = fopen("/spiffs/eeprom_data.bin", "rb");

    if (!f) {
        ESP_LOGI("SPIFFS", "No EEPROM data to send");
        return;
    }

    sensor_sample_t sample;

    ESP_LOGI("SPIFFS", "Sending EEPROM data...");
    int count = 0;

    ESP_LOGI("SPIFFS", "===== START SENDING EEPROM DATA =====");

    while (fread(&sample, sizeof(sensor_sample_t), 1, f) == 1) {
        send_to_influx(sample);
        count++;

        ESP_LOGI("SPIFFS",
                 "Sent [%d]: V=%.2f V | I=%.2f A | P=%.2f W | T=%.2f °C | TS=%lld",
                 count,
                 sample.voltage,
                 sample.current,
                 sample.power,
                 sample.temperature,
                 (long long)sample.timestamp);

        vTaskDelay(pdMS_TO_TICKS(500));
    }

    fclose(f);

    ESP_LOGI("SPIFFS",
             "===== DONE: %d EEPROM samples sent to InfluxDB =====",
             count);

    ESP_LOGI("SPIFFS", "All EEPROM data sent. Clearing file.");

    remove("/spiffs/eeprom_data.bin");
}

void sensor_task(void *pvParameters) {
    sensor_sample_t sample;

    while (1) {
        sample.voltage = getVoltage();
        sample.current = getCurrent();
        sample.power = sample.voltage * sample.current;
        sample.temperature = getTemperature();//max31865
        if (g_panel_config.board_type == BOARD_GY21) {
        sample.ambient_temperature = getAmbientTemperatureGY21(); 
        sample.humidity = getHumidityGY21();          
        }// GY-21
        if (g_panel_config.board_type == BOARD_IRRADIANCE)
        {
            sample.irradiance = getIrradiance();
        }
        sample.timestamp = time(NULL);

        xQueueSend(sensor_queue, &sample, pdMS_TO_TICKS(10)); // push to queue

        vTaskDelay(pdMS_TO_TICKS(2000)); // delay in millisecond
    }
}


void wifi_mqtt_task(void *pvParameters) {
    sensor_sample_t sample;

    while (1) {
        if (xQueueReceive(sensor_queue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {

            ESP_LOGI(TAGMQTT,
                     "Sample: V=%.2fV, I=%.2fA, P=%.2fW, T=%.2f°C, TS=%lld",
                     sample.voltage, sample.current, sample.power,
                     sample.temperature, (long long)sample.timestamp);

            bool wifi_ok = wifi_connected();
            bool day = is_daytime();

            if (wifi_ok && day) {
                ESP_LOGI(TAGMQTT, "Wi-Fi OK + Daytime: Sending data directly to InfluxDB...");
                send_to_influx(sample);
            }

            else if (!wifi_ok) {
                ESP_LOGW(TAGMQTT, "Wi-Fi down: Storing sample to EEPROM");
                
                save_eeprom_data(&sample);
                print_eeprom_data();
            }
            else {
                ESP_LOGI(TAGMQTT, "Normal condition — No EEPROM/MQTT action.");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void serial_task(void *arg)
{
    char input[128];

    while (1)
    {
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            input[strcspn(input, "\r\n")] = 0;  // remove newline

            char String[32], sn[32], location[32];
            int board;

            if (sscanf(input, "set_panel %31s %31s %31s %d", String, sn,location, &board) == 4)
            {
                strcpy(g_panel_config.String, String);
                strcpy(g_panel_config.panel_sn, sn);
                strcpy(g_panel_config.location, location);
                g_panel_config.board_type = board;
                panel_config_save(&g_panel_config);

                ESP_LOGI("CONFIG", "Updated String=%s panel_sn=%s location=%s board_type=%d",
                         g_panel_config.String,
                         g_panel_config.panel_sn,
                         g_panel_config.location,
                         g_panel_config.board_type);
            }
            else if (strcmp(input, "show_panel") == 0)
            {
                ESP_LOGI("CONFIG", "String=%s panel_sn=%s location=%s board_type=%d",
                         g_panel_config.String,
                         g_panel_config.panel_sn,
                         g_panel_config.location,
                         g_panel_config.board_type);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



void app_main(void) {

    // PCB reset-control line: keep high to prevent periodic reset.
    //reset_hold_high();

    last_reset_reason = esp_reset_reason();
    ESP_LOGI(TAGMQTT, "Reset reason: %d", last_reset_reason);

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //configuring panel associated to esp
    if (panel_config_load(&g_panel_config) == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(g_panel_config.String, "Not_Set");
        strcpy(g_panel_config.location, "Not_Set");
        strcpy(g_panel_config.panel_sn, "Not_Set");
        g_panel_config.board_type = BOARD_UNSET;
        panel_config_save(&g_panel_config);
    }

    ESP_LOGI("CONFIG", "String: %s  | SN: %s | Location: %s | Board Type: %d",
             g_panel_config.String,
             g_panel_config.panel_sn,
             g_panel_config.location,
             g_panel_config.board_type);

    //spiffs logs
    init_spiffs();
    print_logs_spiffs();

    buzzer_init();

    wifi_init_sta();        // initialize WiFi
    // obtain_time();          // synchronize time via SNTP
    // mqtt_app_start();       // start MQTT client

    ESP_LOGI("BOOT", "Reset reason: %d", esp_reset_reason());

    int wait_count = 0;
    if (!is_daytime() && wifi_connected()) {
        send_eeprom_data();
        sleep_until_sunrise();
    }

    // Initialize hardware
    i2c_master_init();      // initialize I2C bus
    max31865_init();        // initialize MAX31865
    irradiance_adc_init();  // initialize irradiance ADC

    // Create queue for sensor samples
    sensor_queue = xQueueCreate(20, sizeof(sensor_sample_t));

    // Setup MQTT topic
    //char macID[13];
    get_mac_address(macID, sizeof(macID));
    snprintf(topic, sizeof(topic), "sensor/%s", macID);

    // Create FreeRTOS tasks pinned to cores
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 8192, NULL, 2, NULL, 0);     // Core 0
    xTaskCreatePinnedToCore(wifi_mqtt_task, "wifi_mqtt_task", 12288, NULL, 2, NULL, 1); // Core 1
    xTaskCreatePinnedToCore(serial_task,"serial_task",4096,NULL,1,NULL,1);//task to enter panel details via serial input

    ESP_LOGI(TAGMQTT, "Sensor Task free stack: %u bytes",
             uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
}
