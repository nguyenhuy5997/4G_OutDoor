/*
 * FOTA_LTE.h
 *
 *  Created on: Jun 26, 2022
 *      Author: ASUS
 */
#ifdef __cplusplus
extern "C" {
#endif
#ifndef MAIN_OTA_LTE_FOTA_LTE_H_
#define MAIN_OTA_LTE_FOTA_LTE_H_

#include <esp_ota_ops.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "simcom7600.h"
#include "common.h"

esp_err_t  update_handler();


#endif /* MAIN_OTA_LTE_FOTA_LTE_H_ */
#ifdef __cplusplus
} /* end of "extern C" block */
#endif
