/*Please install esp_arduino core 2.0.3 to run code bellow
 * Author: Huy Dao Nguyen
 * Date: 04.07.2022
 */
#include "common.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "simcom7600.h"
#include "7600_config.h"
#include "common.h"
#include "location_parser.h"
#include "json_user.h"
#include "wifi_cell.h"
#include "FOTA_LTE.h"
#include "Button.h"
#define TAG_MAIN "SIMCOM"
#define BUTTON (gpio_num_t)0

#define CLIENT_ID       "MAN02ND00074"
#define MQTT_BROKER     "tcp://vttmqtt.innoway.vn:1883"
#define CLIENT_PW       "NULL"
#define VERSION         "0.0.1"
#define PUB_TOPIC_GPS   "messages/" CLIENT_ID "/gps"  
#define PUB_TOPIC_WFC   "messages/" CLIENT_ID "/wificell"
#define PUB_TOPIC_SUB   "messages/" CLIENT_ID "/control" 

char wifi_buffer[400];
client mqttClient7600 = {};
gps gps_7600;
Network_Signal network7600 = {};
Device_Infor deviceInfor = {};
LBS LBS_location;
int Location_type = 0x00;
void GetDeviceTimestamp(long *time_stamp)
{
  struct timeval time_now;
  gettimeofday(&time_now, 0);
  *time_stamp = time_now.tv_sec;
}
void init_gpio_output()
{
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)POWER_KEY, 1);
}
void initMqttClient(client* client, const char* id, int sv_type, const char* user_password, const char* broker_mqtt)
{
  client->index = 0;
  memcpy(client->id, id, strlen(id));
  client->sv_type = sv_type;
  memcpy(client->password, user_password, strlen(user_password));
  memcpy(client->broker, broker_mqtt, strlen(broker_mqtt));
}

void subcribe_callback(char * data)
{
  char* _buff;
  _buff = strstr(data, "{");
  sscanf(_buff, "%s", _buff);
  JSON_analyze_sub(_buff, &deviceInfor.Timestamp);
}

void setup()
{
  Serial.begin(115200);
  esp_log_level_set("wifi", ESP_LOG_NONE);
  esp_log_level_set("wifi_init", ESP_LOG_NONE);
  esp_log_level_set("BUTTON", ESP_LOG_NONE);
  nvs_flash_init();
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  sprintf(deviceInfor.Version, "%s", VERSION);
  deviceInfor.Bat_Level = 100;
  init_gpio_output();
  init_simcom(ECHO_UART_PORT_NUM_1, ECHO_TEST_TXD_1, ECHO_TEST_RXD_1, ECHO_UART_BAUD_RATE);
  xTaskCreate(button, "button", 4096, NULL, 3, NULL);
}
void loop()
{
  bool res;
  char pub_mqtt[500];
  while(1)
  {
    POWER_ON:
    printf("----------> START PROGRAM <----------\r\n");

    res = powerOn(POWER_KEY);
    if (res)
    {
      ESP_LOGW(TAG, "Module power on OK");
    }
    else
    {
      ESP_LOGE(TAG, "Module power on FALSE");
      powerOff_(POWER_KEY);
      goto POWER_ON;
    }

    res = isInit(5);
    if(res) ESP_LOGW(TAG, "Module Init OK");
    else ESP_LOGE(TAG, "Module Init FALSE");

    res = echoATSwtich(0, 3);
    if(res) ESP_LOGW(TAG, "Turn off echo OK");
    else ESP_LOGE(TAG, "Turn off echo FALSE");

    res = switchGPS(1, 5);
    if(res) ESP_LOGW(TAG, "Turn on GPS OK");
    else
    {
      ESP_LOGE(TAG, "Turn on GPS FALSE");
      goto POWER_ON;
    }
MQTT:
    res = networkType(BOTH, 3);
    if(res) ESP_LOGW(TAG, "Select network OK");
    else
    {
      ESP_LOGE(TAG, "Select network FALSE");
      if(!isInit(3)) goto POWER_ON;
    }

    res = isRegistered(10);
    if(res) ESP_LOGW(TAG, "Module registed OK");
    else
    {
      if(!isInit(3)) goto POWER_ON;
      ESP_LOGE(TAG, "Module registed FALSE");
    }

    mqttDisconnect(mqttClient7600, 3);

    initMqttClient(&mqttClient7600, CLIENT_ID, 0, CLIENT_PW, MQTT_BROKER);

    res = mqttConnect(mqttClient7600, 5);
    if(res) ESP_LOGW(TAG, "MQTT Connected");
    else
    {
      if(!isInit(3)) goto MQTT;
      ESP_LOGE(TAG, "MQTT can not connect");
    }

    res = mqttSubcribe(mqttClient7600, PUB_TOPIC_SUB, 1, 3, subcribe_callback);
    if(res) ESP_LOGW(TAG, "MQTT Sucribe OK");
    else
    {
      if(!isInit(3))
      {
        ESP_LOGE(TAG, "MQTT Sucribe FALSE");
        goto MQTT;
      }
    }
    res = openNetwork();
    if (res) ESP_LOGW(TAG, "Network Opened");
    else
    {
      if(!isInit(3))
      {
        ESP_LOGE(TAG, "Network Open FALSE");
        goto POWER_ON;
      }
    }
    while(1)
    {
      if (Location_type == 0x01)
      {
        Location_type = 0x00;
        memset(&LBS_location, 0, sizeof(LBS_location));
        res = getLBS(&LBS_location);
        if (res && LBS_location.fix_status)
        {
          gps_7600.GPSfixmode = 2;
          gps_7600.lat = LBS_location.lat;
          gps_7600.lon = LBS_location.lon;
          gps_7600.acc = LBS_location.acc;
          ESP_LOGW(TAG, "LBS get location OK");
        }
        else
        {
          ESP_LOGE(TAG, "LBS get location FALSE");
        }
        networkInfor(5, &network7600);
        MQTT_Location_Payload_Convert(pub_mqtt, gps_7600, network7600,  deviceInfor);
        res = mqttPublish(mqttClient7600, pub_mqtt, PUB_TOPIC_GPS, 1, 1);
        if(res)   ESP_LOGW(TAG, "Publish OK");
        else {
          ESP_LOGE(TAG, "Publish FALSE");
          goto MQTT;
        }
      }
      else if (Location_type == 0x02)
      {
         Location_type = 0x00;
         memset(&gps_7600, 0, sizeof(gps_7600));
         networkInfor(5, &network7600);
         MQTT_Location_Payload_Convert(pub_mqtt, gps_7600, network7600,  deviceInfor);
         res = mqttPublish(mqttClient7600, pub_mqtt, PUB_TOPIC_GPS, 1, 1);
         if(res)   ESP_LOGW(TAG, "Publish OK");
          else {
            ESP_LOGE(TAG, "Publish FALSE");
            goto MQTT;
          }
      }
      else if (Location_type == 0x03)
      {
         Location_type = 0x00;
         networkInfor(5, &network7600);
         wifi_scan(wifi_buffer);
         MQTT_WiFi_Payload_Convert(pub_mqtt, wifi_buffer, network7600, deviceInfor);
         res = mqttPublish(mqttClient7600, pub_mqtt, PUB_TOPIC_WFC, 1, 1);
         memset(wifi_buffer, 0, sizeof(wifi_buffer));
         if(res)   ESP_LOGW(TAG, "Publish OK");
         else {
           ESP_LOGE(TAG, "Publish FALSE");
           goto MQTT;
         }
      }
      vTaskDelay(10/portTICK_PERIOD_MS);
    }
  }
}
void button (void * arg)
{
  button_event_t ev;
  QueueHandle_t button_events = button_init(PIN_BIT(BUTTON));
  int press_count = 0;
  uint32_t capture = 0;
  bool start = false;
  while (true) {
      if ((esp_timer_get_time() / 1000) > (capture + 1000)){
          start = false;
      }
      if (xQueueReceive(button_events, &ev, 1000/portTICK_PERIOD_MS)) {
        if ((ev.pin == BUTTON) && (ev.event == BUTTON_DOWN)) {
          ESP_LOGI(TAG, "Press_count: %d", ++press_count);
          if (press_count == 1){
            start = true;
          }
          capture = esp_timer_get_time() / 1000;
        }
     }
     if (press_count == 1 && start == false){
          Location_type = 0x01;
          press_count = 0;
          ESP_LOGI(TAG, "-------------> LBS <-------------");
        }
     else if (press_count == 2 && start == false ){
        Location_type = 0x02;
        press_count = 0;
        ESP_LOGI(TAG, "-------------> GPS <-------------");
     }
     else if (press_count == 3 && start == false){
        Location_type = 0x03;
        press_count = 0;
        ESP_LOGI(TAG, "-------------> WiFi Cell <-------------");
     }
   }
}
