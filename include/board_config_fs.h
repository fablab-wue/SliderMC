#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool board_config_load_from_fs(void);
bool board_config_save_to_fs(void);

#ifdef __cplusplus
}
#endif
