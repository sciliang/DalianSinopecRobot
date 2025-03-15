#ifndef INC_NMEA_GPTXT_H
#define INC_NMEA_GPTXT_H

#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "nmea/nmea.h"

/* Value indexes */
#define NMEA_GPTXT_ID00 0
#define NMEA_GPTXT_ID01 1
#define NMEA_GPTXT_ID02 2
#define NMEA_GPTXT_TEXT 3
/* Text field length */
#define NMEA_GPTXT_TEXT_SIZE 64

typedef struct {
	nmea_s base;
	int id_00;
	int id_01;
	int id_02;
	char text[NMEA_GPTXT_TEXT_SIZE];
} nmea_gptxt_s;

int nmea_gptxt_init(nmea_parser_s *parser);
int nmea_gptxt_allocate_data(nmea_parser_s *parser);
int nmea_gptxt_set_default(nmea_parser_s *parser);
int nmea_gptxt_free_data(nmea_s *data);
int nmea_gptxt_parse(nmea_parser_s *parser, char *value, int val_index);

#endif  /* INC_NMEA_GPTXT_H */
