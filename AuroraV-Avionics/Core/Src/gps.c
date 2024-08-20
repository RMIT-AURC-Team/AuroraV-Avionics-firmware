/***********************************************************************************
 * @file        gps.c                                                              *
 * @author      Matt Ricci                                                         *
 * @addtogroup  GPS                                                                *
 * @brief       Brief description of the file’s purpose.                           *
 *                                                                                 *
 * @{                                                                              *
 ***********************************************************************************/

#include "gps.h"

uint8_t GPS_byte() {
  while((USART3->SR & USART_SR_RXNE) == 0);
  uint8_t a =(uint8_t)((0X000000FF)&(USART3->DR));
	return a;
}
 
 void GPS_message(char * message){
  //$GPGGA
  uint8_t flag = 0;
  message[0] = '$';
  message[1] = 'G';  
  message[2] = 'N'; 
  message[3] = 'G'; 
  message[4] = 'G'; 
  message[5] = 'A'; 
	 
	while(1){
		for (int x =0; x<6;x++){
			uint8_t byte = GPS_byte();
			if (message[x] != byte){
				flag = 0;
				break;
			}	 
			if(x == 5)
				flag = 1;
		}

		if (flag){
			uint8_t byte =0;
			for(int x = 6; byte != '\n'; x++){
				byte = GPS_byte();
				message[x] = byte;
			}
			return;
		}
	}
	
}
 
void DecodeGPS(char* GPS, struct GPSData* data) {
	int GPSpointer = 7;
	int pointer = 0;
	int tempPointer = 0;
	char* temp[] = { &data->time[0],&data->latitude[0],&data->N_S[0], &data->longitude[0],&data->E_W[0], &data->fix[0], &data->satellites[0], &data->hdop[0], &data->altitude[0]};
	while (tempPointer <= 8) {
		if (GPS[GPSpointer] == ',') {
			if ((pointer == 0) && ((tempPointer ==1)||(tempPointer == 3)))
				data->lock = 0;
			else if ((pointer >= 8) && ((tempPointer == 1) || (tempPointer == 3)))
				data->lock = 1;
			pointer = 0;
			tempPointer++;
		}
		else {
			if (pointer <= 15) {
				temp[tempPointer][pointer] = GPS[GPSpointer];
				temp[tempPointer][pointer + 1] = '\0';
				pointer++;
			}
		}
		GPSpointer++;
	}

	data->hour = (data->time[0] - '0') * 10 + (data->time[1] - '0');
	data->minute = (data->time[2] - '0') * 10 + (data->time[3] - '0');
	data->second = (data->time[4] - '0') * 10 + (data->time[5] - '0');
}

/** @} */