#include "nmea/parsers/gpchc.h"
#include "../parser_types.h"
#include "parse.h"

int nmea_gpchc_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GPCHC);
	NMEA_PARSER_PREFIX(parser, "GPCHC");
	return 0;
}

int nmea_gpchc_allocate_data(nmea_parser_s *parser)
{
	parser->data = malloc(sizeof(nmea_gpchc_s));
	if (NULL == parser->data)
	{
		return -1;
	}

	return 0;
}

int nmea_gpchc_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gpchc_s));
	return 0;
}

int nmea_gpchc_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

int nmea_gpchc_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gpchc_s *data = (nmea_gpchc_s *)parser->data;

	switch (val_index)
	{
	case NMEA_GPCHC_WEEK:
		data->gpchc.gps_week = (uint16_t)atoi(value);
		break;
	case NMEA_GPCHC_SEC:
		data->gpchc.gps_sec = strtof(value, NULL);
		break;
	case NMEA_GPCHC_HEADING:
		data->gpchc.heading = strtof(value, NULL);
		break;
	case NMEA_GPCHC_PITCH:
		data->gpchc.pitch = strtof(value, NULL);
		break;
	case NMEA_GPCHC_ROLL:
		data->gpchc.roll = strtof(value, NULL);
		break;
	case NMEA_GPCHC_GYRO_X:
		data->gpchc.gyro_x = strtof(value, NULL);
		break;
	case NMEA_GPCHC_GYRO_Y:
		data->gpchc.gyro_y = strtof(value, NULL);
		break;
	case NMEA_GPCHC_GYRO_Z:
		data->gpchc.gyro_z = strtof(value, NULL);
		break;
	case NMEA_GPCHC_ACC_X:
		data->gpchc.acc_x = strtof(value, NULL);
		break;
	case NMEA_GPCHC_ACC_Y:
		data->gpchc.acc_y = strtof(value, NULL);
		break;
	case NMEA_GPCHC_ACC_Z:
		data->gpchc.acc_z = strtof(value, NULL);
		break;
	case NMEA_GPCHC_LATITUDE:
		data->gpchc.latitude = strtod(value, NULL);
		break;
	case NMEA_GPCHC_LONGITUDE:
		data->gpchc.longitude = strtod(value, NULL);
		break;
	case NMEA_GPCHC_ALTITUDE:
		data->gpchc.altitude = strtof(value, NULL);
		break;
	case NMEA_GPCHC_VE:
		data->gpchc.velEast = strtof(value, NULL);
		break;
	case NMEA_GPCHC_VN:
		data->gpchc.velNorth = strtof(value, NULL);
		break;
	case NMEA_GPCHC_VU:
		data->gpchc.velUp = strtof(value, NULL);
		break;
	case NMEA_GPCHC_V:
		data->gpchc.velocity = strtof(value, NULL);
		break;
	case NMEA_GPCHC_NSV1:
		data->gpchc.nsv1 = (uint8_t)atoi(value);
		break;
	case NMEA_GPCHC_NSV2:
		data->gpchc.nsv2 = (uint8_t)atoi(value);
		break;
	case NMEA_GPCHC_STATUS:
		data->gpchc.status.r_uint8 = (uint8_t)atoi(value);
		break;
	case NMEA_GPCHC_AGE:
		data->gpchc.status.r_uint8 = (uint8_t)atoi(value);
		break;
	case NMEA_GPCHC_WARNING:
		data->gpchc.status.r_uint8 = (uint8_t)atoi(value);
		break;
	default:
		break;
	}
	return 0;
}