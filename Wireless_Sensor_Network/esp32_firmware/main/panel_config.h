#ifndef PANEL_CONFIG_H
#define PANEL_CONFIG_H

#include "esp_err.h"

typedef struct {
    char panel_label[32];
    char panel_sn[32];
} panel_config_t;

esp_err_t panel_config_init(void);
esp_err_t panel_config_load(panel_config_t *config);
esp_err_t panel_config_save(panel_config_t *config);

#endif