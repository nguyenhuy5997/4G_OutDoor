/*
 * wifi_cell.h
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */
#ifdef __cplusplus
extern "C" {
#endif
#ifndef MAIN_WIFI_CELL_WIFI_CELL_H_
#define MAIN_WIFI_CELL_WIFI_CELL_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include "esp_wifi.h"
#include "cJSON.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "json_user.h"
void wifi_scan(char* Wifi_Buffer);

#endif /* MAIN_WIFI_CELL_WIFI_CELL_H_ */

#ifdef __cplusplus
} /* end of "extern C" block */
#endif
