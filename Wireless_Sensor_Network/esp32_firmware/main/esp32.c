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
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include <stdint.h>
#include "esp_spiffs.h"
#include <sys/stat.h>
#include "cJSON.h"
#include "panel_config.h"

panel_config_t g_panel_config;

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

#define BUZZER_PIN 2


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


typedef struct {
    float voltage;
    float current;
    float power;
    float temperature;
    time_t timestamp;
} sensor_sample_t;

static QueueHandle_t sensor_queue;

#define OTA_URL "http://NEHA.local:8000/esp32.bin"
static const char *TAG_OTA = "OTA";

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

void ota_update_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA update...");

    // First create http client config
    esp_http_client_config_t http_config = {
        .url = OTA_URL,
        .timeout_ms = 10000,
    };

    // Then create OTA config and assign pointer
    esp_https_ota_config_t ota_config = {
        .http_config = &http_config
    };

    // Start OTA
    esp_err_t ret = esp_https_ota(&ota_config);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA update successful, restarting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA update failed! Error 0x%x", ret);
    }

    vTaskDelete(NULL);
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


//  MAX31865 INIT
void max31865_init(void) {
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

        //Initialising SNTP
        if (!sntp_initialized) {
        obtain_time();
        sntp_initialized = true;
    }
    
    //creating a small task to do eeprom data transfer
    xTaskCreate(send_eeprom_task, "send_eeprom_task", 8192, NULL, 5, NULL);
        
    // Start OTA task
    xTaskCreate(&ota_update_task, "ota_task", 8192, NULL, 5, NULL);
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
    char line[256];
    time_t ts = sample.timestamp;

    // Prepare InfluxDB line protocol
    // snprintf(line, sizeof(line),
    //     "solar_data,device=esp32_02 voltage=%.2f,current=%.2f,power=%.2f,temperature=%.2f %lld",
    //     sample.voltage, sample.current, sample.power, sample.temperature, ts
    // );

    snprintf(line, sizeof(line),
        "solar,MCU=%s,panel_label=%s,panel_sn=%s voltage=%.2f,current=%.2f,power=%.2f,temperature=%.2f %lld",
        macID, g_panel_config.panel_label,g_panel_config.panel_sn, sample.voltage, sample.current, sample.power, sample.temperature, ts
    );
    printf("INFLUX LINE >>> %s\n", line);

    // Build full URLe
    char url[256];
    snprintf(url, sizeof(url),
             "%s/api/v2/write?org=%s&bucket=%s&precision=s",
             INFLUX_URL, INFLUX_ORG, INFLUX_BUCKET);

    // HTTP client config
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 10000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    // **Important:** Authorization header must start with "Token "
    char auth_header[512];
    snprintf(auth_header, sizeof(auth_header), "Token %s", INFLUX_TOKEN);
    esp_http_client_set_header(client, "Authorization", auth_header);
    esp_http_client_set_header(client, "Content-Type", "text/plain; charset=utf-8");

    // Set data to POST
    esp_http_client_set_post_field(client, line, strlen(line));

    // Perform request
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        if (status == 204) {
            ESP_LOGI(TAGMQTT, "InfluxDB write OK");
            save_log_spiffs("Data sent to InfluxDB");
            play_data_sent();
            //vTaskDelete(NULL);
        } else {
            ESP_LOGE(TAGMQTT, "InfluxDB HTTP error: %d", status);
        }
    } else {
        ESP_LOGE(TAGMQTT, "InfluxDB POST failed: %s", esp_err_to_name(err));
    }

    // Cleanup
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

//sending data to influxdb as batch
// esp_err_t send_to_influx_batch(const char *payload)
// {
//     esp_http_client_config_t config = {
//         .url = "http://YOUR_INFLUX_URL",
//         .method = HTTP_METHOD_POST,
//         .timeout_ms = 5000,
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&config);

//     esp_http_client_set_header(client, "Content-Type", "text/plain");

//     esp_http_client_set_post_field(client, payload, strlen(payload));

//     esp_err_t err = esp_http_client_perform(client);

//     esp_http_client_cleanup(client);

//     return err;
// }

// //  SAVE SAMPLE TO EEPROM (simulated with RAM buffer) 
// void save_to_eeprom(sensor_sample_t *sample, int index) {
//     if (index < MAX_SAMPLES) {
//         eeprom_buffer[index] = *sample;
//         ESP_LOGI(TAGMQTT, "Saved sample to EEPROM at index %d", index);
//     }
//     // eeprom_index=index;
//     ESP_LOGI(TAGMQTT, "Total sample index noted to be %d",eeprom_index);
// }


// //  READ SAMPLE FROM EEPROM (RAM buffer for simulation) 
// void read_from_eeprom(int index, sensor_sample_t *sample) {
//     if (index < MAX_SAMPLES && sample != NULL) {
//         *sample = eeprom_buffer[index];
//     } else {
//         ESP_LOGW(TAGMQTT, "EEPROM read failed at index %d", index);
//     }
// }

// // SEND ALL EEPROM SAMPLES AFTER SUNSET 
// void send_eeprom_data() {
//     ESP_LOGI(TAGMQTT, "Calling send_eeprom function");
//     ESP_LOGI(TAGMQTT, "Total sample index noted to be %d",eeprom_index);
//     if (eeprom_index == 0) 
//     {
//         ESP_LOGI(TAGMQTT, "Nothing to sent from EEPROM");
//         return; // nothing to send
//     }
//     ESP_LOGI(TAGMQTT, "Sending %d stored samples from EEPROM...", eeprom_index);
//     for (int i = 0; i < eeprom_index; i++) {
//         send_to_influx(eeprom_buffer[i]);
//         vTaskDelay(pdMS_TO_TICKS(500)); // small delay between messages
//     }
//     ESP_LOGI(TAGMQTT, "All stored EEPROM data sent. Clearing buffer.");
//     save_log_spiffs("All data from eeprom sent");
//     eeprom_index = 0; // clear buffer
// }

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

//function to send all eeprom data in one batch instead of multiple https requets
// void send_eeprom_data()
// {
//     ESP_LOGI(TAGMQTT, "Calling send_eeprom function");
//     ESP_LOGI(TAGMQTT, "Total sample index noted to be %d", eeprom_index);

//     if (eeprom_index == 0) {
//         ESP_LOGI(TAGMQTT, "Nothing to send from EEPROM");
//         return;
//     }

//     char payload[2048];  // adjust size if needed
//     memset(payload, 0, sizeof(payload));

//     ESP_LOGI(TAGMQTT, "Preparing batch payload...");

//     for (int i = 0; i < eeprom_index; i++) {

//         char line[256];

//         snprintf(line, sizeof(line),
//             "solar_data,device=esp32_01 voltage=%.2f,current=%.2f,power=%.2f,temperature=%.2f %lld\n",
//             eeprom_buffer[i].voltage,
//             eeprom_buffer[i].current,
//             eeprom_buffer[i].power,
//             eeprom_buffer[i].temperature,
//             (long long)eeprom_buffer[i].timestamp
//         );

//         // Prevent buffer overflow
//         if (strlen(payload) + strlen(line) < sizeof(payload)) {
//             strcat(payload, line);
//         } else {
//             ESP_LOGE(TAGMQTT, "Payload too large! Sending partial batch...");
//             break;
//         }
//     }

//     ESP_LOGI(TAGMQTT, "Sending batch to Influx...");

//     // Send once
//     int status = send_to_influx_batch(payload);

//     if (status == ESP_OK) {
//         ESP_LOGI(TAGMQTT, "Batch sent successfully. Clearing EEPROM.");

//         save_log_spiffs("All EEPROM data sent");

//         eeprom_index = 0;
//     } else {
//         ESP_LOGE(TAGMQTT, "Failed to send batch. Keeping data in EEPROM.");
//     }
// }

void sensor_task(void *pvParameters) {
    sensor_sample_t sample;

    while (1) {
        sample.voltage = getVoltage();
        sample.current = getCurrent();
        sample.power = sample.voltage * sample.current;
        sample.temperature = getTemperature();
        sample.timestamp = time(NULL);

        xQueueSend(sensor_queue, &sample, pdMS_TO_TICKS(10)); // push to queue

        vTaskDelay(pdMS_TO_TICKS(2000)); // delay in millisecond
    }
}

//obtaining sensor values by averaging
// void sensor_task(void *pvParameters)
// {
//     while (1)
//     {
//         float voltage_sum = 0;
//         float current_sum = 0;
//         float temp_sum = 0;

//         int samples = 4;

//         for (int i = 0; i < samples; i++) {
//             voltage_sum += getVoltage();
//             current_sum += getCurrent();
//             temp_sum += getTemperature();

//             vTaskDelay(pdMS_TO_TICKS(1000)); // 1 reading/sec
//         }

//         sensor_sample_t sample;

//         sample.voltage = voltage_sum / samples;
//         sample.current = current_sum / samples;
//         sample.temperature = temp_sum / samples;
//         sample.power = sample.voltage * sample.current;
//         sample.timestamp = time(NULL);

//         xQueueSend(sensor_queue, &sample, pdMS_TO_TICKS(10));
//     }
// }



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

            

            // // else if (!wifi_ok && day) {
            // else if (!wifi_ok) {
            //     ESP_LOGW(TAGMQTT, "Wi-Fi down (Daytime): Storing sample to EEPROM...");

            //     if (eeprom_index < MAX_SAMPLES) {
            //         //save_to_eeprom(&sample, eeprom_index++);
            //         save_eeprom_data(&sample);
            //     } else {
            //         ESP_LOGW(TAGMQTT, "EEPROM full, ignoring sample");
            //     }

            //     // Immediately read back to verify
            //     sensor_sample_t readback;
            //     for (int i = 0; i < eeprom_index; i++) {
            //         read_from_eeprom(i, &readback);
            //         ESP_LOGI(TAGMQTT,
            //                  "EEPROM[%d]: V=%.2f, I=%.2f, P=%.2f, T=%.2f, TS=%lld",
            //                  i, readback.voltage, readback.current,
            //                  readback.power, readback.temperature,
            //                  (long long)readback.timestamp);
            //     }
            // } 

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

            char label[32], sn[32];

            if (sscanf(input, "set_panel %31s %31s", label, sn) == 2)
            {
                strcpy(g_panel_config.panel_label, label);
                strcpy(g_panel_config.panel_sn, sn);

                panel_config_save(&g_panel_config);

                ESP_LOGI("CONFIG", "Updated panel_label=%s panel_sn=%s",
                         g_panel_config.panel_label,
                         g_panel_config.panel_sn);
            }
            else if (strcmp(input, "show_panel") == 0)
            {
                ESP_LOGI("CONFIG", "panel_label=%s panel_sn=%s",
                         g_panel_config.panel_label,
                         g_panel_config.panel_sn);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



void app_main(void) {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    //configuring panel associated to esp

    if (panel_config_load(&g_panel_config) == ESP_ERR_NVS_NOT_FOUND) {
        strcpy(g_panel_config.panel_label, "Not_Set");
        strcpy(g_panel_config.panel_sn, "Not_Set");
        panel_config_save(&g_panel_config);
    }

    ESP_LOGI("CONFIG", "Panel Label: %s | SN: %s",
             g_panel_config.panel_label,
             g_panel_config.panel_sn);

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
