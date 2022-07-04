/*
 * location_parser.c
 *
 *  Created on: 10 Jun 2022
 *      Author: nguyenphuonglinh
 */
#include "location_parser.h"

int filter_comma(char *respond_data, int begin, int end, char *output)
{
	memset(output, 0, strlen(output));
	int count_filter = 0, lim = 0, start = 0, finish = 0,i;
	for (i = 0; i < strlen(respond_data); i++)
	{
		if ( respond_data[i] == ',')
		{
			count_filter ++;
			if (count_filter == begin)			start = i+1;
			if (count_filter == end)			finish = i;
		}

	}
	lim = finish - start;
	for (i = 0; i < lim; i++){
		output[i] = respond_data[start];
		start ++;
	}
	output[i] = 0;
	return 0;
}
bool CPSI_Decode(char* str, Network_Signal *Device_Singnal)
{
	if(strstr(str,"GSM"))
	{
		char temp_buf[50] = {0};
		char _LAC[10];
		char RX_buf[10];
		int i = 0, head = 0, tail = 0, k = 0, index = 0;
		for(i = 0; i < strlen(str); i++)
		{
			if(str[i] == ',') ++k;
			if(k == 1) head = i;
			if(k == 6) tail = i;
		}
		for(i = head+2; i < tail+1; i++)
		{
			if(str[i] == '-') str[i] = ',';
			temp_buf[index++] = str[i];
		}
		sscanf(temp_buf, "%d,%d,%[^,],%d,%[^,],,%d", &Device_Singnal->MCC, &Device_Singnal->MNC, _LAC, &Device_Singnal->cell_ID, RX_buf, &Device_Singnal->RSSI);
		Device_Singnal->RSSI = -1 * (Device_Singnal->RSSI);
		filter_comma(str,4,5,_LAC);
		Device_Singnal->LAC = (int)strtol(_LAC, NULL, 0);
		Device_Singnal->Network_type = GSM;
		if(Device_Singnal->MCC && Device_Singnal->MNC && Device_Singnal->cell_ID && Device_Singnal->RSSI && Device_Singnal->LAC)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else if(strstr(str,"LTE"))
	{
		char temp_buf[50] = {0};
		char _LAC[10];
		char RSRP_Buf[10];
		char RSRQ_Buf[10];
		int i = 0, head1 = 0, tail1 = 0, head2 = 0, tail2 = 0, k = 0, index = 0;
		for(i = 0; i < strlen(str); i++)
		{
			if(str[i] == ',') ++k;
			if(k == 1) head1 = i;
			if(k == 4) tail1 = i;
			if(k == 10) head2 = i;
			if(k == 11) tail2 = i;
		}
		for(i = head1 + 2; i < tail1 + 1; i++)
		{
			if(str[i] == '-') str[i] = ',';
			temp_buf[index++] = str[i];
		}
		for(i= head2 + 1; i < tail2 + 1; i++)
		{
			temp_buf[index++] = str[i];
		}
		sscanf(temp_buf, "%d,%d,%[^,],%d,%d", &Device_Singnal->MCC, &Device_Singnal->MNC, _LAC, &Device_Singnal->cell_ID, &Device_Singnal->RSSI);
		Device_Singnal->LAC = (int)strtol(_LAC, NULL, 0);
		filter_comma(str, 11, 12, RSRQ_Buf);
		Device_Singnal->RSRQ = atoi(RSRQ_Buf)/10;
		filter_comma(str, 12, 13, RSRP_Buf);
		Device_Singnal->RSRP = atoi(RSRP_Buf)/10;
		Device_Singnal->Network_type = LTE;
		if(Device_Singnal->MCC && Device_Singnal->MNC && Device_Singnal->cell_ID && Device_Singnal->RSSI && Device_Singnal->LAC && Device_Singnal->RSRQ && Device_Singnal->RSRP)
		{

			return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}

void getGPS(char *getGSPraw, gps *gps_7600){
	char    GPSraw[128];
	char    GPSdata[32][20];
	char    GPSdatedata[16], GPStimedata[16];
	char    GPSnorth, GPSeast;
	float   RAWlat = 0, RAWlon = 0, GPSlat = 0, GPSlong = 0;
	int     GPSfixmode = 0;
	int     GPSyear, GPSmonth, GPSdate;
	int     GPShours, GPSminutes, GPSseconds;
	int     GPSs = 0, GLONASSs = 0, BEIDOUs = 0 ;
	float   accuracy;
	float   GPSaltitude = 0, GPSspeed = 0;
	int     GPSready = 0;
	int     vsat, usat;
	int     t = 0, n = 0, i = 0;
	strcpy (GPSraw, getGSPraw);
	int len = strlen(GPSraw);
	for (i = 0; i <= len; i++){
		if (GPSraw[i] != ',') {
			if (n <= 16) {
				GPSdata[n][t] = GPSraw[i];
				t++;
			}
		}
		else {
			GPSdata[n][t] = '\0';
			n++; t = 0;
		}
	}
	GPSfixmode = (int)GPSdata[0][0] - 48;
	gps_7600->GPSfixmode = GPSfixmode;
	if (GPSfixmode > 1) {
		GPSs        = atoi(GPSdata[1]);
		GLONASSs    = atoi(GPSdata[2]);
		BEIDOUs     = atoi(GPSdata[3]);
		RAWlat      = atof(GPSdata[4]);
		GPSnorth    = GPSdata[5][0];
		RAWlon      = atof(GPSdata[6]);
		GPSeast     = GPSdata[7][0];
		strcpy(GPSdatedata,  GPSdata[8]);
		strcpy(GPStimedata,  GPSdata[9]);
		GPSaltitude = atof(GPSdata[10]);
		GPSspeed    = atof(GPSdata[11]);
		accuracy    = atof(GPSdata[13]);
		if (RAWlat && RAWlon)
		{
			GPSlat = (floor(RAWlat / 100) + fmod(RAWlat, 100.) / 60) *
					(GPSnorth == 'N' ? 1 : -1);
			GPSlong = (floor(RAWlon / 100) + fmod(RAWlon, 100.) / 60) *
					(GPSeast == 'E' ? 1 : -1);
			GPSspeed = GPSspeed * 1.852;
			sscanf(GPSdatedata, "%2d%2d%4d", &GPSdate, &GPSmonth, &GPSyear);
			sscanf(GPStimedata, "%2d%2d%2d", &GPShours, &GPSminutes, &GPSseconds);
			usat = GPSs + GLONASSs + BEIDOUs;
			vsat = usat;
			GPSready = 1;
		}
	}
	if(GPSready){
		printf("\n\nLat: %3.8f, Long: %3.8f\n", GPSlat, GPSlong);
		printf("Speed: %3.2fKph  Alti: %4.2fM\n", GPSspeed, GPSaltitude);
		printf("Visible Satellites: %d  Used Satellites: %d\n", vsat, usat);
		printf("Accuracy: %2.1f\n", accuracy);
		printf("Date: %d-%d-%d\n", GPSyear, GPSmonth, GPSdate);
		printf("Time: %d:%d:%d\n", GPShours, GPSminutes, GPSseconds);
		gps_7600->lat = GPSlat;
		gps_7600->lon = GPSlong;
		gps_7600->speed = GPSspeed;
		gps_7600->alt = GPSaltitude;
		gps_7600->acc = accuracy;
		gps_7600->bei_num = BEIDOUs;
		gps_7600->glo_num = GLONASSs;
		gps_7600->gps_num = GPSs;
		gps_7600->vsat = usat;
		if (GPSyear < 2000) {
			GPSyear += 2000;
		}
		struct tm s;
		s.tm_sec    = (GPSseconds);
		s.tm_min    = (GPSminutes);
		s.tm_hour   = (GPShours);
		s.tm_mday   = (GPSdate);
		s.tm_mon    = (GPSmonth - 1);
		s.tm_year   = (GPSyear - 1900);
		time_t epoch;
		epoch = mktime(&s);
		if (epoch == -1) {
			//
		}
		else {
			gps_7600->epoch = epoch;
			printf("epoch: %ld\r\n", gps_7600->epoch);
		}
	}
}
void MQTT_Location_Payload_Convert(char* payload_str, gps hgps, Network_Signal Device_signal, Device_Infor Device_infor)
{
	memset(payload_str, 0, 500);
	char Network_Type_Str[10];
	if (Device_signal.Network_type == LTE)
	{
		sprintf(Network_Type_Str, "nb");
	}
	else if(Device_signal.Network_type == GSM)
	{
		sprintf(Network_Type_Str, "2g");
	}
	if(Device_signal.Network_type == GSM)
	{
		sprintf(payload_str, "{\"Type\":\"DPOS\",\"V\":\"%s\",\"ss\":%d,\"r\":%d,\"B\":%d,\"Cn\":\"%s\",\"Acc\":%f,\"lat\":%f,\"lon\":%f,\"T\":%ld}", \
				Device_infor.Version, Device_signal.RSSI, Device_signal.RSRQ, Device_infor.Bat_Level, Network_Type_Str, hgps.acc, hgps.lat, hgps.lon, hgps.epoch);
	}
	else
	{
		sprintf(payload_str, "{\"Type\":\"DPOS\",\"V\":\"%s\",\"ss\":%d,\"r\":%d,\"B\":%d,\"Cn\":\"%s\",\"Acc\":%f,\"lat\":%f,\"lon\":%f,\"T\":%ld}",\
				Device_infor.Version, Device_signal.RSRP, Device_signal.RSRQ, Device_infor.Bat_Level, Network_Type_Str, hgps.acc, hgps.lat, hgps.lon, hgps.epoch);
	}
}
void MQTT_WiFi_Payload_Convert(char* payload_str, char* wifi, Network_Signal Device_signal, Device_Infor Device_infor)
{
	memset(payload_str, 0, 500);
	char Network_Type_Str[10];
	if (Device_signal.Network_type == LTE)
	{
		sprintf(Network_Type_Str, "nb");
	}
	else if(Device_signal.Network_type == GSM)
	{
		sprintf(Network_Type_Str, "2g");
	}
	if(Device_signal.Network_type == GSM)
	{
		sprintf(payload_str,"{\"ss\":%d,\"Type\":\"DWFC\",\"B\":%d,\"r\":%d,\"C\":[{\"C\":%d,\"S\":%d,\"ID\":%d,\"L\":%d,\"N\":%d}],\"T\":%ld,\"V\":\"%s\",",\
				Device_signal.RSSI, Device_infor.Bat_Level, Device_signal.RSRQ, Device_signal.MCC, Device_signal.RSSI, Device_signal.cell_ID, Device_signal.LAC, \
				Device_signal.MNC, Device_infor.Timestamp , Device_infor.Version);
	}
	else
	{
		sprintf(payload_str,"{\"ss\":%d,\"Type\":\"DWFC\",\"B\":%d,\"r\":%d,\"C\":[{\"C\":%d,\"S\":%d,\"ID\":%d,\"L\":%d,\"N\":%d}],\"T\":%ld,\"V\":\"%s\",", \
				Device_signal.RSRP, Device_infor.Bat_Level, Device_signal.RSRQ,  Device_signal.MCC, Device_signal.RSSI,  Device_signal.cell_ID, Device_signal.LAC,\
				Device_signal.MNC, Device_infor.Timestamp, Device_infor.Version);
	}
	if(strlen(wifi) > 0)
	{
		sprintf(payload_str+strlen(payload_str),"%s",wifi);
	}
	if(Device_signal.Network_type == GSM)
	{
		if(strlen(wifi) > 0)
		{
			sprintf(payload_str+strlen(payload_str),"%s",",\"Cn\":\"2g\"}");
		}
		else
		{
			sprintf(payload_str+strlen(payload_str),"%s","\"W\":[],\"Cn\":\"2g\"}");
		}
	}
	else if(Device_signal.Network_type == LTE)
	{
		if(strlen(wifi) > 0)
		{
			sprintf(payload_str+strlen(payload_str),"%s",",\"Cn\":\"nb\"}");
		}
		else
		{
			sprintf(payload_str+strlen(payload_str),"%s","\"W\":[],\"Cn\":\"nb\"}");
		}
	}
}
