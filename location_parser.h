/*
 * location_parser.h
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */
#ifdef __cplusplus
extern "C" {
#endif
#ifndef STRING_USER_LOCATION_PARSER_H_
#define STRING_USER_LOCATION_PARSER_H_
#include "common.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <sys/time.h>
#include <stdbool.h>

int filter_comma(char *respond_data, int begin, int end, char *output);
bool CPSI_Decode(char* str, Network_Signal *Device_Singnal);
void getGPS(char *getGSPraw, gps *gps_7600);
void MQTT_Location_Payload_Convert(char* payload_str, gps hgps, Network_Signal Device_signal, Device_Infor Device_infor);
void MQTT_WiFi_Payload_Convert(char* payload_str, char *wifi_buffer, Network_Signal Device_signal, Device_Infor Device_infor);
#endif /* STRING_USER_LOCATION_PARSER_H_ */
#ifdef __cplusplus
} /* end of "extern C" block */
#endif
