#include "nmea/parsers/gpfpd.h"
#include "../parser_types.h"
#include "parse.h"

int nmea_gpfpd_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GPFPD);
	NMEA_PARSER_PREFIX(parser, "GPFPD");
	return 0;
}

int nmea_gpfpd_allocate_data(nmea_parser_s *parser)
{
	parser->data = malloc(sizeof(nmea_gpfpd_s));
	if (NULL == parser->data)
	{
		return -1;
	}

	return 0;
}

int nmea_gpfpd_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gpfpd_s));
	return 0;
}

int nmea_gpfpd_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

int nmea_gpfpd_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gpfpd_s *data = (nmea_gpfpd_s *)parser->data;

	switch (val_index)
	{
	case NMEA_GPFPD_WEEK:
		data->gpfpd.gps_week = (uint16_t)atoi(value);
		break;
	case NMEA_GPFPD_SEC:
		data->gpfpd.gps_sec = strtof(value, NULL);
		break;
	case NMEA_GPFPD_HEADING:
		data->gpfpd.heading = strtof(value, NULL);
		break;
	case NMEA_GPFPD_PITCH:
		data->gpfpd.pitch = strtof(value, NULL);
		break;
	case NMEA_GPFPD_ROLL:
		data->gpfpd.roll = strtof(value, NULL);
		break;
	case NMEA_GPFPD_LATITUDE:
		data->gpfpd.latitude = strtod(value, NULL);
		break;
	case NMEA_GPFPD_LONGITUDE:
		data->gpfpd.longitude = strtod(value, NULL);
		break;
	case NMEA_GPFPD_ALTITUDE:
		data->gpfpd.altitude = strtof(value, NULL);
		break;
	case NMEA_GPFPD_VE:
		data->gpfpd.velEast = strtof(value, NULL);
		break;
	case NMEA_GPFPD_VN:
		data->gpfpd.velNorth = strtof(value, NULL);
		break;
	case NMEA_GPFPD_VU:
		data->gpfpd.velUp = strtof(value, NULL);
		break;
	case NMEA_GPFPD_BASELINE:
		data->gpfpd.baseline = strtof(value, NULL);
		break;
	case NMEA_GPFPD_NSV1:
		data->gpfpd.nsv1 = (unsigned char)atoi(value);
		break;
	case NMEA_GPFPD_NSV2:
		data->gpfpd.nsv2 = (unsigned char)atoi(value);
		break;
	case NMEA_GPFPD_STATUS:
		data->gpfpd.status.r_uint8 = (unsigned char)atoi(value);
		break;
	default:
		break;
	}

	return 0;
}