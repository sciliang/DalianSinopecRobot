#ifndef INC_NMEA_GTIMU_H
#define INC_NMEA_GTIMU_H

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "nmea/nmea.h"

typedef struct
{
	uint16_t gps_week; // 自1980-1-6至当前的星期数（格林尼治时间）
	float gps_sec; // 自本周日0:00:00至当前的秒数（格林尼治时间）
	double gyro_x; //deg/s
	double gyro_y; //deg/s
	double gyro_z; //deg/s
	double acc_x; //g
	double acc_y; //g
	double acc_z; //g
	float temperature; //degC
} nmea_gtimu;

typedef struct {
	nmea_s base;
	nmea_gtimu gtimu;
} nmea_gtimu_s;

/* Value indexes */
#define NMEA_GTIMU_WEEK			0
#define NMEA_GTIMU_TIME			1
#define NMEA_GTIMU_GYRO_X		2
#define NMEA_GTIMU_GYRO_Y		3
#define NMEA_GTIMU_GYRO_Z		4
#define NMEA_GTIMU_ACC_X		5
#define NMEA_GTIMU_ACC_Y		6
#define NMEA_GTIMU_ACC_Z		7
#define NMEA_GTIMU_TEMP			8

int nmea_gtimu_init(nmea_parser_s *parser);
int nmea_gtimu_allocate_data(nmea_parser_s *parser);
int nmea_gtimu_set_default(nmea_parser_s *parser);
int nmea_gtimu_free_data(nmea_s *data);
int nmea_gtimu_parse(nmea_parser_s *parser, char *value, int val_index);

#endif  /* INC_NMEA_GTIMU_H */