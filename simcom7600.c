/*
 * simcom7600.c
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */


#include "simcom7600.h"

void uart_simcom(void *arg);
void _sendAT(const char *AT_command);
bool _mqttStart(int retry);
bool _accquireClient(client clientMqtt, int retry);
AT_res ___readSerial(uint32_t timeout, const char* expect);
bool _inputPub(client clientMqtt, const char* topic, int retry);
bool _readSerial(uint32_t timeout);

simcom simcom_7600 = {};
void init_simcom(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baud_rate)
{
	simcom_7600.uart_num = uart_num;
	simcom_7600.tx_io_num = tx_io_num;
	simcom_7600.rx_io_num = rx_io_num;
	simcom_7600.baud_rate = baud_rate;
	uart_config_t uart_config =
	{
		.baud_rate = baud_rate,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	int intr_alloc_flags = 0;
	ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_io_num, rx_io_num, ECHO_TEST_RTS, ECHO_TEST_CTS));
	xTaskCreate(uart_simcom, "uart_echo_task1", 4096, NULL, 10, NULL);
}
void uart_simcom(void *arg)
{
	uint8_t data[BUF_SIZE];
	while (1) {
		int len = uart_read_bytes(simcom_7600.uart_num, data, (BUF_SIZE - 1), 100 / portTICK_PERIOD_MS);
		// Write data back to the UART
		if (len) {
			data[len] = '\0';
			ESP_LOGI(TAG, "Rec: %s", (char*) data);
			if(strstr((char*)data, "+CMQTTRXSTART"))
			{
				simcom_7600.mqtt_CB(data);
			}
			else
			{
				memcpy(simcom_7600.AT_buff, data, len);
				simcom_7600.AT_buff_avai = true;
			}
		}
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}
void _sendAT(const char *AT_command)
{
	ESP_LOGI(TAG, "Send: %s", AT_command);
	simcom_7600.AT_buff_avai = false;
	memset(simcom_7600.AT_buff, 0, BUF_SIZE);
	uart_write_bytes(simcom_7600.uart_num, (const char *) AT_command, strlen((char *)AT_command));
	uart_write_bytes(simcom_7600.uart_num, (const char *)"\r\n", strlen("\r\n"));
	vTaskDelay(100/portTICK_PERIOD_MS);
}
AT_res ___readSerial(uint32_t timeout, const char* expect)
{
	uint64_t timeOld = esp_timer_get_time() / 1000;
	while (!(esp_timer_get_time() / 1000 > timeOld + timeout))
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
		if(simcom_7600.AT_buff_avai)
		{
			if(strstr((char*)simcom_7600.AT_buff, "ERROR")) return AT_ERROR;
			else if(strstr((char*)simcom_7600.AT_buff, expect)) return AT_OK;
		}
	}
	return AT_TIMEOUT;
}
bool _readSerial(uint32_t timeout)
{
	uint64_t timeOld = esp_timer_get_time() / 1000;
	while (!simcom_7600.AT_buff_avai && !(esp_timer_get_time() / 1000 > timeOld + timeout))
	{
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
	if(simcom_7600.AT_buff_avai == false) return false;
	else return true;
}
bool isInit(int retry)
{
	AT_res res;
	while(retry--)
	{
		_sendAT("AT");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool waitModuleReady(int timeout)
{
	AT_res res;
	simcom_7600.AT_buff_avai = false;
	memset(simcom_7600.AT_buff, 0, BUF_SIZE);
	res = ___readSerial(timeout, "PB DONE");
	if (res == AT_OK) return true;
	else if (res == AT_ERROR) return false;
	return false;
}
bool switchGPS(bool enable, int retry)
{
	AT_res res;
	char buff[10];
	sprintf(buff, "AT+CGPS=%d", enable);
	while(retry--)
	{
		_sendAT(buff);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool readGPS(gps *gps)
{
	char *gps_buff;
	_sendAT("AT+CGNSSINFO");
	if(_readSerial(5000) == false) return false;
	else
	{
		gps_buff = strtok((char*)simcom_7600.AT_buff, " ");
		gps_buff = strtok(NULL, "");
		gps_buff = strtok(gps_buff, "\r\n");
		getGPS(gps_buff, gps);
		return true;
	}
}
bool powerOff(int retry)
{
	AT_res res;
	while(retry--)
	{
		_sendAT("AT+CPOF");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool isRegistered(int retry)
{
	while(retry--)
	{
		vTaskDelay(1000/portTICK_PERIOD_MS);
		_sendAT("AT+CREG?");
		if(_readSerial(1000) == false) continue;
		if(strstr((char*)simcom_7600.AT_buff, "0,1") || strstr((char*)simcom_7600.AT_buff, "0,5") || strstr((char*)simcom_7600.AT_buff, "1,1") || strstr((char*)simcom_7600.AT_buff, "1,5")) return true;
		else continue;
	}
	return false;
}
bool _mqttStart(int retry)
{
	AT_res res;
	while(retry--)
	{
		_sendAT("AT+CMQTTSTART");
		res = ___readSerial(10000, "+CMQTTSTART: 0");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool _accquireClient(client clientMqtt, int retry)
{
	AT_res res;
	char buff[100];
	sprintf(buff, "AT+CMQTTACCQ=%d,\"%s\",%d", clientMqtt.index, clientMqtt.id, clientMqtt.sv_type);
	while (retry--)
	{
		_sendAT(buff);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool mqttConnect(client clientMqtt, int retry)
{
	AT_res _res;
	bool res = false;
	res = _mqttStart(2);
	if (!res) return false;
	res = _accquireClient(clientMqtt, 3);
	if (!res) return false;
	char buff_send[200];
	sprintf(buff_send, "AT+CMQTTCONNECT=%d,\"%s\",60,1,\"%s\",\"%s\"", clientMqtt.index, clientMqtt.broker, clientMqtt.id, clientMqtt.password);
	char buff_exp[20];
	sprintf(buff_exp, "CMQTTCONNECT: %d,0", clientMqtt.index);
	while (retry--)
	{
		_sendAT(buff_send);
		_res = ___readSerial(10000, buff_exp);
		if (_res == AT_OK) return true;
		else if (_res == AT_ERROR) return false;
	}
	return false;
}
bool _inputPub(client clientMqtt, const char* topic, int retry)
{
	AT_res res;
	int _retry = retry;
	char buff_send[200];
	sprintf(buff_send, "AT+CMQTTTOPIC=%d,%d", clientMqtt.index, strlen(topic));
	while(_retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, ">");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	_retry = retry;
	while(_retry--)
	{
		_sendAT(topic);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
void mqttDisconnect(client clientMqtt, int retry)
{
	AT_res res;
	char buff_send[50], buff_exp[20];
	int _retry = retry;
	sprintf(buff_send, "AT+CMQTTDISC=%d,60", clientMqtt.index);
	sprintf(buff_exp, "+CMQTTDISC: %d,0", clientMqtt.index);
	while(_retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, buff_exp);
		if (res == AT_OK) break;
		else if (res == AT_ERROR) break;
	}
	_retry = retry;
	sprintf(buff_send, "AT+CMQTTREL=%d", clientMqtt.index);
	while(_retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) break;
	}
	_retry = retry;
	while(_retry--)
	{
		_sendAT("AT+CMQTTSTOP");
		res = ___readSerial(1000, "+CMQTTSTOP: 0");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) break;
	}
}
bool mqttPublish(client clientMqtt, const char* data, const char* topic, int qos, int retry)
{
	AT_res _res;
	int _retry = retry;
	bool res = false;
	char buff_send[200];
	char buff_exp[20];
	res = _inputPub(clientMqtt, topic, 3);
	if(!res) return false;
	sprintf(buff_send, "AT+CMQTTPAYLOAD=%d,%d", clientMqtt.index, strlen(data));
	while(_retry--)
	{
		_sendAT(buff_send);
		_res = ___readSerial(1000, ">");
		if (_res == AT_OK) break;
		else if (_res == AT_ERROR) return false;
	}
	_retry = retry;
	while(_retry--)
	{
		_sendAT(data);
		_res = ___readSerial(1000, "OK");
		if (_res == AT_OK) break;
		else if (_res == AT_ERROR) return false;
	}
	_retry = retry;
	sprintf(buff_send, "AT+CMQTTPUB=%d,%d,60,0", clientMqtt.index, qos);
	sprintf(buff_exp, "+CMQTTPUB: %d,0", clientMqtt.index);
	while(_retry--)
	{
		_sendAT(buff_send);
		_res = ___readSerial(5000, buff_exp);
		if (_res == AT_OK) return true;
		else if (_res == AT_ERROR) return false;
	}
	return false;
}
bool mqttSubcribe(client clientMqtt, const char* topic, int qos, int retry, void (*mqttSubcribeCB)(char * data))
{
	AT_res res;
	int _retry = retry;
	char buff_send[50];
	char buff_exp[20];
	sprintf(buff_send, "AT+CMQTTSUBTOPIC=%d,%d,%d", clientMqtt.index, strlen(topic), qos);
	while(_retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, ">");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	_retry = retry;
	while(_retry--)
	{
		_sendAT(topic);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	_retry = retry;
	sprintf(buff_send, "AT+CMQTTSUB=%d", clientMqtt.index);
	sprintf(buff_exp, "+CMQTTSUB: %d,0", clientMqtt.index);
	while(_retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(5000, buff_exp);
		if (res == AT_OK)
		{
			simcom_7600.mqtt_CB = mqttSubcribeCB;
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool echoATSwtich(bool enable, int retry)
{
	AT_res res;
	char buff_send[5];
	sprintf(buff_send, "ATE%d", enable);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool networkType(network net_type, int retry)
{
	AT_res res;
	char buff_send[20];
	if (net_type == GSM) sprintf(buff_send, "AT+CNMP=%d", 13);
	else if (net_type == LTE) sprintf(buff_send, "AT+CNMP=%d", 38);
	else if (net_type == BOTH) sprintf(buff_send, "AT+CNMP=%d", 2);
	while(retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool sendSMS(const char *phone, const char *text)
{
	AT_res res;
	char buff_send[100];
	int retry = 3;
	while (retry--)
	{
		_sendAT("AT+CMGF=1");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 3;
	sprintf(buff_send, "AT+CMGS=\"%s\"", phone);
	while (retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, ">");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 3;
	while (retry--)
	{
		char data[] = { 0x1A };
		_sendAT(text);
		uart_write_bytes(simcom_7600.uart_num, data, sizeof(data));
		res = ___readSerial(10000, "OK");
		if (res == AT_OK) return true;
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool networkInfor(int retry, Network_Signal* network)
{
	AT_res res;
	while (retry--)
	{
		_sendAT("AT+CPSI?");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK)
		{
			CPSI_Decode((char*)simcom_7600.AT_buff, network);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
void powerOff_(gpio_num_t powerKey)
{
	gpio_set_level(powerKey, 1);
	vTaskDelay(200/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 0);
	vTaskDelay(5000/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 1);
}
bool powerOn(gpio_num_t powerKey)
{
	AT_res res;
	gpio_set_level(powerKey, 1);
	vTaskDelay(200/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 0);
	vTaskDelay(500/portTICK_PERIOD_MS);
	gpio_set_level(powerKey, 1);
	res = waitModuleReady(16000);
	if (res) return true;
	else return false;
}
bool httpGet(const char * url, uint32_t* len)
{
	AT_res res;
	int retry = 2;
	char buff_send[100];
	sprintf(buff_send, "AT+HTTPPARA=\"URL\",\"%s\"", url);
	while(retry--)
	{
		_sendAT("AT+HTTPINIT");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 2;
	while (retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK) break;
		else if (res == AT_ERROR) return false;
	}
	retry = 2;
	while(retry--)
	{
		_sendAT("AT+HTTPACTION=0");
		res = ___readSerial(5000, "+HTTPACTION: 0,200");
		if (res == AT_OK)
		{
			sscanf((char*)simcom_7600.AT_buff, "\r\n+HTTPACTION: 0,200,%d\r\n\r\nOK\r\n", len);
			printf("content_lengt: %d\r\n", *len);
			sscanf((char*)simcom_7600.AT_buff, "\r\nOK\r\n\r\n+HTTPACTION: 0,200,%d\r\n\r\nOK\r\n", len);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}

bool httpReadRespond(uint8_t* data, int len_expect, uint16_t *len_real)
{
	AT_res res;
	int retry = 2;
	char buff_send[30];
	sprintf(buff_send, "AT+HTTPREAD=%d", len_expect);
	while (retry--)
	{
		_sendAT(buff_send);
		res = ___readSerial(1000, "OK");
		if (res == AT_OK)
		{
			int read_len;
			sscanf((char*)simcom_7600.AT_buff, "\r\nOK\r\n\r\n+HTTPREAD: DATA,%d", &read_len);
			*len_real = read_len;
			uint8_t buff_temp[read_len+strlen("\r\n+HTTPREAD:0\r\n")];
			int index_CRLF = 0;
			for(int i = 0; i < BUF_SIZE; i++)
			{
				if(*(simcom_7600.AT_buff+i-1) == '\n' && *(simcom_7600.AT_buff+i-2) == '\r')
				{
					index_CRLF++;
				}
				if(index_CRLF == 4)
				{
					memcpy(buff_temp, simcom_7600.AT_buff+i, read_len+strlen("\r\n+HTTPREAD:0\r\n"));
					break;
				}
			}
//			buff_temp[strlen(buff_temp) - strlen("\r\n+HTTPREAD:0\r\n")] = '\0';
			memcpy(data, buff_temp, read_len);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool checkPDPstate(int *PDP_state)
{
	AT_res res;
	int retry = 2;
	char sub_buf[2];
	while (retry--)
	{
		_sendAT("AT+CNETSTART?");
		res = ___readSerial(1000, "OK");
		if (res == AT_OK)
		{
			getSubStrig((char*)simcom_7600.AT_buff, "+CNETSTART: ", "\r\n", sub_buf);
			*PDP_state = atoi(sub_buf);
			return true;
		}
		else if (res == AT_ERROR) return false;
	}
	return false;
}
bool openNetwork()
{
	AT_res res;
	int retry = 2;
	int PDP_state;
	checkPDPstate(&PDP_state);
	if (PDP_state == 2) return true;
	else
	{
		while (retry--)
		{
			_sendAT("AT+CNETSTART");
			res = ___readSerial(2000, "OK");
			if (res == AT_OK)
			{
				if (strstr((char*)simcom_7600.AT_buff, "+CNETSTART: 0"))
				{
					return true;
				}
				else return false;
			}
		}
	}
	return false;
}
bool getLBS(LBS *LBS_infor)
{
	AT_res res;
	int retry = 2;
	char buf_temp[50];
	while (retry--)
	{
		_sendAT("AT+CLBS=1");
		res = ___readSerial(10000, "+CLBS:");
		if (res == AT_OK)
		{
			if (strstr((char*)simcom_7600.AT_buff, "+CLBS: 0"))
			{
				float lat, lon;
				uint16_t acc;
				getSubStrig((char*)simcom_7600.AT_buff, "+CLBS: 0,", "\r\n", buf_temp);
				sscanf(buf_temp, "%f,%f,%"SCNd16"", &LBS_infor->lat, &LBS_infor->lon, &LBS_infor->acc);
				LBS_infor->fix_status = 1;
				return true;
			}
			else return false;
		}
	}
	return false;
}
