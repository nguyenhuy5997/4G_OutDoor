/*
 * simcom7600.h
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */
#ifdef __cplusplus
extern "C" {
#endif
#ifndef SIMCOM7600_SIMCOM7600_H_
#define SIMCOM7600_SIMCOM7600_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include "common.h"
#include <inttypes.h>
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
#include "simcom7600.h"
#include "7600_config.h"
#include "location_parser.h"
#include "string_parse.h"
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)
#define ECHO_TASK_STACK_SIZE    (2048)
#define BUF_SIZE (2048)

typedef struct simcom_t
{
	uart_port_t uart_num;
	int tx_io_num;
	int rx_io_num;
	int baud_rate;
	bool AT_buff_avai;
	uint8_t AT_buff[BUF_SIZE];
	void (*mqtt_CB)(char * data);
}simcom;
typedef struct client_t
{
	char password[20];
	char id[20];
	char broker[50];
	int	index;
	int sv_type;
}client;
typedef struct LBS_t
{
	float lat;
	float lon;
	uint16_t acc;
	bool fix_status;
}LBS;
typedef enum
{
	AT_OK,
	AT_ERROR,
	AT_TIMEOUT,
}AT_res;


void init_simcom(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baud_rate);
bool isInit(int retry);
bool echoATSwtich(bool enable, int retry);
bool waitModuleReady(int timeout);
bool powerOff(int retry);
void powerOff_(gpio_num_t powerKey);
bool powerOn(gpio_num_t powerKey);
bool networkType(network net_type, int retry);
bool isRegistered(int retry);
bool networkInfor(int retry, Network_Signal* network);
bool switchGPS(bool enable, int retry);
bool readGPS(gps *gps);
bool mqttConnect(client clientMqtt, int retry);
void mqttDisconnect(client clientMqtt, int retry);
bool mqttPublish(client clientMqtt, const char* data, const char* topic, int qos, int retry);
bool mqttSubcribe(client clientMqtt, const char * topic, int qos, int retry, void (*mqttSubcribeCB)(char *data));
bool sendSMS(const char *phone, const char *text);
bool httpGet(const char * url, uint32_t* len);
bool httpReadRespond(uint8_t* data, int len_expect, uint16_t *len_real);
bool checkPDPstate(int *PDP_state);
bool openNetwork();
bool getLBS(LBS *LBS_infor);
#endif /* SIMCOM7600_SIMCOM7600_H_ */
#ifdef __cplusplus
} /* end of "extern C" block */
#endif
