#ifndef INC_NMEA_GPGGA_H
#define INC_NMEA_GPGGA_H

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "nmea/nmea.h"

typedef struct {
	struct tm time;
	nmea_position longitude;
	nmea_position latitude;
	int n_satellites;
	double altitude;
	char altitude_unit;
	double undulation;
	char undulation_unit;
	unsigned char position_fix;
} nmea_gpgga;

typedef struct {
	nmea_s base;
	nmea_gpgga gpgga;
} nmea_gpgga_s;

/* Value indexes */
#define NMEA_GPGGA_TIME			0
#define NMEA_GPGGA_LATITUDE		1
#define NMEA_GPGGA_LATITUDE_CARDINAL	2
#define NMEA_GPGGA_LONGITUDE		3
#define NMEA_GPGGA_LONGITUDE_CARDINAL	4
#define NMEA_GPGGA_POSITION_FIX		5
#define NMEA_GPGGA_N_SATELLITES		6
#define NMEA_GPGGA_ALTITUDE		8
#define NMEA_GPGGA_ALTITUDE_UNIT	9
#define NMEA_GPGGA_UNDULATION		10
#define NMEA_GPGGA_UNDULATION_UNIT	11

#define INVALID_UNDULATION -9999.999

int nmea_gpgga_init(nmea_parser_s *parser);
int nmea_gpgga_allocate_data(nmea_parser_s *parser);
int nmea_gpgga_set_default(nmea_parser_s *parser);
int nmea_gpgga_free_data(nmea_s *data);
int nmea_gpgga_parse(nmea_parser_s *parser, char *value, int val_index);

#endif  /* INC_NMEA_GPGGA_H */
