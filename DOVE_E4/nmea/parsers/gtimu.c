#include "nmea/parsers/gtimu.h"
#include "../parser_types.h"
#include "parse.h"

int nmea_gtimu_init(nmea_parser_s *parser)
{
	/* Declare what sentence type to parse */
	NMEA_PARSER_TYPE(parser, NMEA_GTIMU);
	NMEA_PARSER_PREFIX(parser, "GTIMU");
	return 0;
}

int nmea_gtimu_allocate_data(nmea_parser_s *parser)
{
	parser->data = malloc(sizeof(nmea_gtimu_s));
	if (NULL == parser->data)
	{
		return -1;
	}

	return 0;
}

int nmea_gtimu_set_default(nmea_parser_s *parser)
{
	memset(parser->data, 0, sizeof(nmea_gtimu_s));
	return 0;
}

int nmea_gtimu_free_data(nmea_s *data)
{
	free(data);
	return 0;
}

int nmea_gtimu_parse(nmea_parser_s *parser, char *value, int val_index)
{
	nmea_gtimu_s *data = (nmea_gtimu_s *)parser->data;

	switch (val_index)
	{
	case NMEA_GTIMU_WEEK:
		data->gtimu.gps_week = (uint16_t)atoi(value);
		break;
	case NMEA_GTIMU_TIME:
		data->gtimu.gps_sec = strtof(value, NULL);
		break;
	case NMEA_GTIMU_GYRO_X:
		data->gtimu.gyro_x = strtod(value, NULL);
		break;
	case NMEA_GTIMU_GYRO_Y:
		data->gtimu.gyro_y = strtod(value, NULL);
		break;
	case NMEA_GTIMU_GYRO_Z:
		data->gtimu.gyro_z = strtod(value, NULL);
		break;
	case NMEA_GTIMU_ACC_X:
		data->gtimu.acc_x = strtod(value, NULL);
		break;
	case NMEA_GTIMU_ACC_Y:
		data->gtimu.acc_y = strtod(value, NULL);
		break;
	case NMEA_GTIMU_ACC_Z:
		data->gtimu.acc_z = strtod(value, NULL);
		break;
	case NMEA_GTIMU_TEMP:
		data->gtimu.temperature = strtof(value, NULL);
		break;
	default:
		break;
	}

	return 0;
}