/*
 * FOTA_LTE.c
 *
 *  Created on: Jun 26, 2022
 *      Author: ASUS
 */

#ifndef MAIN_OTA_LTE_FOTA_LTE_C_
#define MAIN_OTA_LTE_FOTA_LTE_C_

#include "FOTA_LTE.h"

#define SEND_DATA   1024
esp_err_t  update_handler(){
//	esp_log_level_set(TAG, ESP_LOG_NONE);
	const char* TAG_OTA = "OTA_LTE";
    esp_err_t ret;
    uint32_t total_download = 0;
    int threshold_percent_log = 1;
    const esp_partition_t *update_partition;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running  = esp_ota_get_running_partition();
    esp_ota_handle_t well_done_handle = 0;
    if (configured != running) {
        ESP_LOGW(TAG_OTA, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                        configured->address, running->address);
        ESP_LOGW(TAG_OTA, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)"
                        );
    }
    ESP_LOGI(TAG_OTA, "Running partition type %d subtype %d (offset 0x%08x)",
                    running->type, running->subtype, running->address);

    ESP_LOGI(TAG_OTA, "Starting OTA example");

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG_OTA, "Writing to partition subtype %d at offset 0x%x",
                    update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    ESP_ERROR_CHECK(esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &well_done_handle));

    uint32_t content_len;
	uint8_t buf[SEND_DATA];
	httpGet("http://202.191.56.104:5551/uploads/ESP7600.bin", &content_len);
    ESP_LOGI(TAG_OTA, "Image szie: %d", content_len);
	uint16_t buf_len = 0;
    while(httpReadRespond(buf, SEND_DATA, &buf_len)){

        ret = esp_ota_write(well_done_handle, buf, buf_len);
        memset(buf, 0, sizeof(buf));

        total_download = total_download +  buf_len;
        if((total_download * 100)/content_len > threshold_percent_log)
        {
        	threshold_percent_log++;
        	ESP_LOGI(TAG_OTA, "FOTA progress: %d%%", (total_download * 100)/content_len);
        }
        if(ret != ESP_OK){
            ESP_LOGE(TAG, "Firmware upgrade failed");
            while (1) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            return ESP_FAIL;
        }
    }

    ESP_ERROR_CHECK(esp_ota_end(well_done_handle));

    ESP_ERROR_CHECK(esp_ota_set_boot_partition(update_partition));
    ESP_LOGI(TAG, "Restarting...");
    esp_restart();
    return ESP_OK;
}


#endif /* MAIN_OTA_LTE_FOTA_LTE_C_ */
