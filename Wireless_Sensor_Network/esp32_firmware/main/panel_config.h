#ifndef PANEL_CONFIG_H
#define PANEL_CONFIG_H

#include "esp_err.h"

typedef enum {
    BOARD_UNSET = 0,
    BOARD_NORMAL = 1,
    BOARD_GY21 = 2,
    BOARD_IRRADIANCE = 3
} board_type_t;

typedef struct {
    char String[32];
    char panel_sn[32];
    char location[32];
    board_type_t board_type;
} panel_config_t;

esp_err_t panel_config_init(void);
esp_err_t panel_config_load(panel_config_t *config);
esp_err_t panel_config_save(panel_config_t *config);

#endif