#include "nmea/nmea.h"
#include "parser.h"

#define DECLARE_PARSER_API(modname)                                   \
	extern int nmea_##modname##_init(nmea_parser_s *parser);          \
	extern int nmea_##modname##_allocate_data(nmea_parser_s *parser); \
	extern int nmea_##modname##_set_default(nmea_parser_s *parser);   \
	extern int nmea_##modname##_free_data(nmea_s *data);              \
	extern int nmea_##modname##_parse(nmea_parser_s *parser, char *value, int val_index);

#define PARSER_LOAD(modname)                                  \
	parser = &(parsers[i]);                                   \
	parser->handle = NULL;                                    \
	parser->allocate_data = nmea_##modname##_allocate_data;   \
	parser->set_default = nmea_##modname##_set_default;       \
	parser->free_data = nmea_##modname##_free_data;           \
	parser->parse = nmea_##modname##_parse;                   \
	if (-1 == nmea_##modname##_init((nmea_parser_s *)parser)) \
	{                                                         \
		return -1;                                            \
	}                                                         \
	i++;

#ifdef ENABLE_GPGGA
DECLARE_PARSER_API(gpgga)
#endif
#ifdef ENABLE_GPGLL
DECLARE_PARSER_API(gpgll)
#endif
#ifdef ENABLE_GPGSA
DECLARE_PARSER_API(gpgsa)
#endif
#ifdef ENABLE_GPGSV
DECLARE_PARSER_API(gpgsv)
#endif
#ifdef ENABLE_GPRMC
DECLARE_PARSER_API(gprmc)
#endif
#ifdef ENABLE_GPTXT
DECLARE_PARSER_API(gptxt)
#endif
#ifdef ENABLE_GPVTG
DECLARE_PARSER_API(gpvtg)
#endif
#ifdef ENABLE_GPFPD
DECLARE_PARSER_API(gpfpd)
#endif
#ifdef ENABLE_GTIMU
DECLARE_PARSER_API(gtimu)
#endif
#ifdef ENABLE_GPCHC
DECLARE_PARSER_API(gpchc)
#endif

nmea_parser_module_s parsers[PARSER_COUNT];

nmea_parser_module_s *
nmea_init_parser(const char *filename)
{
	/* This function intentionally returns NULL */
	return NULL;
}

int nmea_load_parsers()
{
	int i = 0;
	nmea_parser_module_s *parser;

#ifdef ENABLE_GPGGA
	PARSER_LOAD(gpgga);
#endif
#ifdef ENABLE_GPGLL
	PARSER_LOAD(gpgll);
#endif
#ifdef ENABLE_GPGSA
	PARSER_LOAD(gpgsa);
#endif
#ifdef ENABLE_GPGSV
	PARSER_LOAD(gpgsv);
#endif
#ifdef ENABLE_GPRMC
	PARSER_LOAD(gprmc);
#endif
#ifdef ENABLE_GPTXT
	PARSER_LOAD(gptxt);
#endif
#ifdef ENABLE_GPVTG
	PARSER_LOAD(gpvtg);
#endif
#ifdef ENABLE_GPFPD
	PARSER_LOAD(gpfpd);
#endif
#ifdef ENABLE_GTIMU
	PARSER_LOAD(gtimu);
#endif
#ifdef ENABLE_GPCHC
	PARSER_LOAD(gpchc);
#endif

	return PARSER_COUNT;
}

void nmea_unload_parsers()
{
	/* This function body is intentionally left empty,
		 because there is no dynamic memory allocations. */
}

nmea_parser_module_s *
nmea_get_parser_by_type(nmea_t type)
{
	int i;

	for (i = 0; i < PARSER_COUNT; i++)
	{
		if (type == parsers[i].parser.type)
		{
			return &(parsers[i]);
		}
	}

	return (nmea_parser_module_s *)NULL;
}

nmea_parser_module_s *
nmea_get_parser_by_sentence(const char *sentence)
{
	int i;
	char *Replace_N = "P";
	for (i = 0; i < PARSER_COUNT; i++)
	{
		memcpy(sentence + 2, Replace_N, 1); // replace N -->P
		if (0 == strncmp(sentence + 1, parsers[i].parser.type_word, NMEA_PREFIX_LENGTH))
		{
			return &(parsers[i]);
		}
	}

	return (nmea_parser_module_s *)NULL;
}
