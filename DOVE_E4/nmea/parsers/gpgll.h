#ifndef INC_NMEA_GPGLL_H
#define INC_NMEA_GPGLL_H

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "nmea/nmea.h"

typedef struct {
	nmea_s base;
	nmea_position longitude;
	nmea_position latitude;
	struct tm time;
} nmea_gpgll_s;

/* Value indexes */
#define NMEA_GPGLL_LATITUDE		0
#define NMEA_GPGLL_LATITUDE_CARDINAL	1
#define NMEA_GPGLL_LONGITUDE		2
#define NMEA_GPGLL_LONGITUDE_CARDINAL	3
#define NMEA_GPGLL_TIME			4

int nmea_gpgll_init(nmea_parser_s *parser);
int nmea_gpgll_allocate_data(nmea_parser_s *parser);
int nmea_gpgll_set_default(nmea_parser_s *parser);
int nmea_gpgll_free_data(nmea_s *data);
int nmea_gpgll_parse(nmea_parser_s *parser, char *value, int val_index);

#endif  /* INC_NMEA_GPGLL_H */
