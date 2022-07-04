/*
 * wifi_cell.c
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */

#include "wifi_cell.h"
#define TAG "ESP_WIFI"
#define DEFAULT_SCAN_LIST_SIZE 10

void wifi_scan(char* Wifi_Buffer)
{
	uint16_t ap_count;
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		esp_wifi_init(&cfg);
		char *buffer = NULL;
		int err = 0;
		cJSON *root_obj = cJSON_CreateObject();
		cJSON *aps_obj = cJSON_CreateArray();
		if (root_obj == NULL)
		{
			ESP_LOGE(TAG, "Create JSON object fail!\r\n");
		}
		uint16_t number = DEFAULT_SCAN_LIST_SIZE;
		wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
		esp_wifi_set_mode(WIFI_MODE_STA);
		esp_wifi_start();
		esp_wifi_scan_start(NULL, true);
		esp_wifi_scan_get_ap_records(&number, ap_info);
		esp_wifi_scan_get_ap_num(&ap_count);
		ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);

		if(ap_count > 0)
		{
			for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++)
			{
				char BSSID_Buf[20];
				sprintf(BSSID_Buf, "%x:%x:%x:%x:%x:%x", ap_info[i].bssid[0], ap_info[i].bssid[1], ap_info[i].bssid[2], ap_info[i].bssid[3], ap_info[i].bssid[4], ap_info[i].bssid[5]);
				err |= json_add_ap(aps_obj, BSSID_Buf, ap_info[i].rssi);
			}
			err |= json_add_obj(root_obj, "W", aps_obj);
			if (err)
			{
				ESP_LOGE(TAG,"There are some error when making JSON object!\r\n");
			}
			else
			{
				buffer = cJSON_PrintUnformatted(root_obj);
				cJSON_Delete(root_obj);
			}
			if (buffer == NULL)
			{
				ESP_LOGE(TAG,"There are no data to send!\r\n");
			}
			else
			{
				ESP_LOGE(TAG, "Print wifi buffer \r\n");
				ESP_LOGW(TAG,"%s",buffer);
				ESP_LOGE(TAG, "Allocate memory for wifi buffer \r\n");
				ESP_LOGE(TAG, "Copy wifi buffer \r\n");
				for(int i = 1; i < strlen(buffer) - 1; i++)
				{
					Wifi_Buffer[i-1] = buffer[i];
				}
			}
		}
		else
		{
			memset(Wifi_Buffer, 0, strlen(Wifi_Buffer));
		}
		(esp_wifi_scan_stop());
		(esp_wifi_stop());
		(esp_wifi_deinit());
}

