#ifndef INC_NMEA_GPCHC_H
#define INC_NMEA_GPCHC_H

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "nmea.h"

typedef struct
{
	uint16_t gps_week; // 自1980-1-6至当前的星期数（格林尼治时间）
	float gps_sec;	   // 自本周日0:00:00至当前的秒数（格林尼治时间
	float heading;	   // deg
	float pitch;	   // deg
	float roll;		   // deg
	float gyro_x;	   // deg/s
	float gyro_y;	   // deg/s
	float gyro_z;	   // deg/s
	float acc_x;	   // g
	float acc_y;	   // g
	float acc_z;	   // g
	double latitude;   // deg
	double longitude;  // deg
	float altitude;	   // deg
	float velEast;	   // m/s
	float velNorth;	   // m/s
	float velUp;	   // m/s
	float velocity;	   // m/s
	uint8_t nsv1;
	uint8_t nsv2;

	union
	{
		uint8_t r_uint8;
		struct
		{
			/*0 初始化
			 * 1 卫导模式
			 * 2 组合导航模式
			 * 3 纯惯导模式
			 */
			uint8_t system : 4;
			/*0：不定位不定向
		   1：单点定位定向
		   2：伪距差分定位定向
		   3：组合推算
		   4：RTK稳定解定位定向
		   5：RTK浮点解定位定向
		   6：单点定位不定向
		   7：伪距差分定位不定向
		   8：RTK稳定解定位不定向
		   9：RTK浮点解定位不定向
		   */
			uint8_t rtk : 4;
		} val;
	} status;

	uint8_t diff_age; // 差分延时

	union
	{
		uint8_t r_uint8;
		struct
		{
			uint8_t gps_chars : 1;	// 1:无GPS消息，0：正常
			uint8_t veh_chars : 1;	// 1:无车辆消息，0：正常
			uint8_t gyro_error : 1; // 1:陀螺错误，0：正常
			uint8_t acc_error : 1;	// 1:加表错误，0：正常
			uint8_t rev : 4;
		} val;
	} warning;
} nmea_gpchc;

typedef struct
{
	nmea_s base;
	nmea_gpchc gpchc;
} nmea_gpchc_s;

/* Value indexes */
#define NMEA_GPCHC_WEEK 0
#define NMEA_GPCHC_SEC 1
#define NMEA_GPCHC_HEADING 2
#define NMEA_GPCHC_PITCH 3
#define NMEA_GPCHC_ROLL 4
#define NMEA_GPCHC_GYRO_X 5
#define NMEA_GPCHC_GYRO_Y 6
#define NMEA_GPCHC_GYRO_Z 7
#define NMEA_GPCHC_ACC_X 8
#define NMEA_GPCHC_ACC_Y 9
#define NMEA_GPCHC_ACC_Z 10
#define NMEA_GPCHC_LATITUDE 11
#define NMEA_GPCHC_LONGITUDE 12
#define NMEA_GPCHC_ALTITUDE 13
#define NMEA_GPCHC_VE 14
#define NMEA_GPCHC_VN 15
#define NMEA_GPCHC_VU 16
#define NMEA_GPCHC_V 17
#define NMEA_GPCHC_NSV1 18
#define NMEA_GPCHC_NSV2 19
#define NMEA_GPCHC_STATUS 20
#define NMEA_GPCHC_AGE 21
#define NMEA_GPCHC_WARNING 22

int nmea_gpchc_init(nmea_parser_s *parser);
int nmea_gpchc_allocate_data(nmea_parser_s *parser);
int nmea_gpchc_set_default(nmea_parser_s *parser);
int nmea_gpchc_free_data(nmea_s *data);
int nmea_gpchc_parse(nmea_parser_s *parser, char *value, int val_index);

#endif /* INC_NMEA_GPCHC_H */