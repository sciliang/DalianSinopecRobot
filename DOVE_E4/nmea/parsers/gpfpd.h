#ifndef INC_NMEA_GPFPD_H
#define INC_NMEA_GPFPD_H

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "nmea/nmea.h"

typedef struct
{
	uint16_t gps_week; // 自1980-1-6至当前的星期数（格林尼治时间）
	float gps_sec; // 自本周日0:00:00至当前的秒数（格林尼治时间）
	float heading; //deg
	float pitch; //deg
	float roll; //deg
	double latitude; //deg
	double longitude; //deg
	float altitude; //deg
	float velEast; //m/s
	float velNorth; //m/s
	float velUp; //m/s
	float baseline; //m
	unsigned char nsv1;
	unsigned char nsv2;

	union
	{
		uint8_t r_uint8;
		struct
		{
			uint8_t system : 4; //see enum SystemStatus
			uint8_t rtk : 4; //see enum RtkStatus
		} val;
	} status;
} nmea_gpfpd;

typedef struct {
	nmea_s base;
	nmea_gpfpd gpfpd;
} nmea_gpfpd_s;


/* Value indexes */
#define NMEA_GPFPD_WEEK			0
#define NMEA_GPFPD_SEC			1
#define NMEA_GPFPD_HEADING		2
#define NMEA_GPFPD_PITCH		3
#define NMEA_GPFPD_ROLL			4
#define NMEA_GPFPD_LATITUDE		5
#define NMEA_GPFPD_LONGITUDE	6
#define NMEA_GPFPD_ALTITUDE		7
#define NMEA_GPFPD_VE			8
#define NMEA_GPFPD_VN			9
#define NMEA_GPFPD_VU			10
#define NMEA_GPFPD_BASELINE		11
#define NMEA_GPFPD_NSV1			12
#define NMEA_GPFPD_NSV2			13
#define NMEA_GPFPD_STATUS		14

int nmea_gpfpd_init(nmea_parser_s *parser);
int nmea_gpfpd_allocate_data(nmea_parser_s *parser);
int nmea_gpfpd_set_default(nmea_parser_s *parser);
int nmea_gpfpd_free_data(nmea_s *data);
int nmea_gpfpd_parse(nmea_parser_s *parser, char *value, int val_index);

#endif  /* INC_NMEA_GPFPD_H */