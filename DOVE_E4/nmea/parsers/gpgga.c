#include "nmea/parsers/gpgga.h"
#include "../parser_types.h"
#include "parse.h"

int nmea_gpgga_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GPGGA);
	NMEA_PARSER_PREFIX(parser, "GPGGA");
	return 0;
}

int nmea_gpgga_allocate_data(nmea_parser_s *parser)
{
	parser->data = malloc(sizeof(nmea_gpgga_s));
	if (NULL == parser->data)
	{
		return -1;
	}

	return 0;
}

int nmea_gpgga_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gpgga_s));
	// Set the default undulation to an invalid value
	nmea_gpgga_s *data = (nmea_gpgga_s *)parser->data;
	data->gpgga.undulation = INVALID_UNDULATION;
	return 0;
}

int nmea_gpgga_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

int nmea_gpgga_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gpgga_s *data = (nmea_gpgga_s *)parser->data;

	switch (val_index)
	{
	case NMEA_GPGGA_TIME:
		/* Parse time */
		if (-1 == nmea_time_parse(value, &data->gpgga.time))
		{
			return -1;
		}
		break;

	case NMEA_GPGGA_LATITUDE:
		/* Parse latitude */
		if (-1 == nmea_position_parse(value, &data->gpgga.latitude))
		{
			return -1;
		}
		break;

	case NMEA_GPGGA_LATITUDE_CARDINAL:
		/* Parse cardinal direction */
		data->gpgga.latitude.cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->gpgga.latitude.cardinal)
		{
			return -1;
		}
		break;

	case NMEA_GPGGA_LONGITUDE:
		/* Parse longitude */
		if (-1 == nmea_position_parse(value, &data->gpgga.longitude))
		{
			return -1;
		}
		break;

	case NMEA_GPGGA_LONGITUDE_CARDINAL:
		/* Parse cardinal direction */
		data->gpgga.longitude.cardinal = nmea_cardinal_direction_parse(value);
		if (NMEA_CARDINAL_DIR_UNKNOWN == data->gpgga.longitude.cardinal)
		{
			return -1;
		}
		break;

	case NMEA_GPGGA_POSITION_FIX:
		/* Parse position fix indicator */
		data->gpgga.position_fix = atoi(value);
		break;

	case NMEA_GPGGA_N_SATELLITES:
		/* Parse number of satellies */
		data->gpgga.n_satellites = atoi(value);
		break;

	case NMEA_GPGGA_ALTITUDE:
		/* Parse altitude */
		data->gpgga.altitude = atof(value);
		break;

	case NMEA_GPGGA_ALTITUDE_UNIT:
		/* Parse altitude unit */
		data->gpgga.altitude_unit = *value;
		break;

	case NMEA_GPGGA_UNDULATION:
		/* Parse undulation */
		data->gpgga.undulation = atof(value);
		break;

	case NMEA_GPGGA_UNDULATION_UNIT:
		/* Parse undulation unit */
		data->gpgga.undulation_unit = *value;
		break;

	default:
		break;
	}

	return 0;
}
