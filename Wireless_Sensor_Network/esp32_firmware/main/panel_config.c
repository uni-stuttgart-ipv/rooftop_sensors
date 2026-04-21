#include "panel_config.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

#define NAMESPACE "panel_cfg"
#define KEY "panel_info"


esp_err_t panel_config_save(panel_config_t *config)
{
    nvs_handle_t handle;
    nvs_open(NAMESPACE, NVS_READWRITE, &handle);

    nvs_set_blob(handle, KEY, config, sizeof(panel_config_t));
    nvs_commit(handle);
    nvs_close(handle);

    return ESP_OK;
}

esp_err_t panel_config_load(panel_config_t *config)
{
    nvs_handle_t handle;
    size_t size = sizeof(panel_config_t);

    nvs_open(NAMESPACE, NVS_READWRITE, &handle);
    esp_err_t err = nvs_get_blob(handle, KEY, config, &size);
    nvs_close(handle);

    return err;
}