#ifndef INC_NMEA_PARSER_TYPES_H
#define INC_NMEA_PARSER_TYPES_H

#include "nmea/nmea.h"

#define NMEA_PARSER_PREFIX(parser, type_prefix) memcpy(parser->type_word, type_prefix, NMEA_PREFIX_LENGTH)
#define NMEA_PARSER_TYPE(parser, nmea_type) parser->type = nmea_type


#endif  /* INC_NMEA_PARSER_TYPES_H */
