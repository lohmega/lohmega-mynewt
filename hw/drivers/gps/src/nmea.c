#include <stdio.h>
#include <math.h>
#include "gps/gps.h"
#include "gps/nmea.h"

#define ARRAY_LENGTH(a) (sizeof a / sizeof (a[0]))

#define NMEA_GPRMC_LATITUDE		2
#define NMEA_GPRMC_LATITUDE_CARDINAL	3
#define NMEA_GPRMC_LONGITUDE		4
#define NMEA_GPRMC_LONGITUDE_CARDINAL	5
#define NMEA_GPRMC_TIME			0
#define NMEA_GPRMC_DATE			8
//        TIME      LAT          LONG             N       ALT M
// $GNGGA,102849.00,2729.40086,S,15256.99552,E,1,06,2.95,58.7,M,38.5,M,,*66
#define NMEA_GPGGA_TIME			0
#define NMEA_GPGGA_LATITUDE		1
#define NMEA_GPGGA_LATITUDE_CARDINAL	2
#define NMEA_GPGGA_LONGITUDE		3
#define NMEA_GPGGA_LONGITUDE_CARDINAL	4
#define NMEA_GPGGA_N_SATELLITES		6
#define NMEA_GPGGA_ALTITUDE		8
#define NMEA_GPGGA_ALTITUDE_UNIT	9

#define NMEA_GPGLL_LATITUDE		0
#define NMEA_GPGLL_LATITUDE_CARDINAL	1
#define NMEA_GPGLL_LONGITUDE		2
#define NMEA_GPGLL_LONGITUDE_CARDINAL	3
#define NMEA_GPGLL_TIME			4

#define in_range(c, lo, up)  ((uint8_t)c >= lo && (uint8_t)c <= up)
#define isdigit(c)           in_range(c, '0', '9')

/**
 * Check if a value is not NULL and not empty.
 *
 * Returns 0 if set, otherwise -1.
 */
static int
_is_value_set(const char *value)
{
	if (NULL == value || '\0' == *value) {
		return -1;
	}

	return 0;
}

/**
 * Crop a sentence from the type word and checksum.
 *
 * The type word at the beginning along with the dollar sign ($) will be
 * removed. If there is a checksum, it will also be removed. The two end
 * characters (usually <CR><LF>) will not be included in the new string.
 *
 * sentence is a validated NMEA sentence string.
 * length is the char length of the sentence string.
 *
 * Returns pointer (char *) to the new string.
 */
static char *
_crop_sentence(char *sentence, size_t length)
{
	/* Skip type word, 7 characters (including $ and ,) */
	sentence += NMEA_PREFIX_LENGTH + 2;

	/* Null terminate before end of line/sentence, 2 characters */
	//sentence[length - 9] = '\0';

	/* Remove checksum, if there is one */
	//if ('*' == sentence[length - 12]) {
	//	sentence[length - 12] = '\0';
	//}

	return sentence;
}

/**
 * Splits a string by comma.
 *
 * string is the string to split, will be manipulated. Needs to be
 *        null-terminated.
 * values is a char pointer array that will be filled with pointers to the
 *        splitted values in the string.
 * max_values is the maximum number of values to be parsed.
 *
 * Returns the number of values found in string.
 */
static int
_split_string_by_comma(char *string, char **values, int max_values)
{
	int i = 0;

    char *end = strchr(string, '*');
	values[i++] = string;
	while (i < max_values && NULL != (string = strchr(string, ','))) {
        if (string > end)    break; // End of checksum
		*string = '\0';
		values[i++] = ++string;
        if (*string == '\r') break; // End of CR
        if (*string == '\n') break; // End of NL
	}

	return i;
}

nmea_cardinal_t
nmea_cardinal_direction_parse(char *s)
{
	if (NULL == s || '\0'== *s) {
		return NMEA_CARDINAL_DIR_UNKNOWN;
	}

	switch (*s) {
	case NMEA_CARDINAL_DIR_NORTH:
		return NMEA_CARDINAL_DIR_NORTH;
	case NMEA_CARDINAL_DIR_EAST:
		return NMEA_CARDINAL_DIR_EAST;
	case NMEA_CARDINAL_DIR_SOUTH:
		return NMEA_CARDINAL_DIR_SOUTH;
	case NMEA_CARDINAL_DIR_WEST:
		return NMEA_CARDINAL_DIR_WEST;
	default:
		break;
	}

	return NMEA_CARDINAL_DIR_UNKNOWN;
}

uint8_t
nmea_get_checksum(const char *sentence)
{
	const char *n = sentence + 1;
	uint8_t chk = 0;

	/* While current char isn't '*' or sentence ending (newline) */
	while ('*' != *n && NMEA_END_CHAR_1 != *n && '\0' != *n) {
		chk ^= (uint8_t) *n;
		n++;
	}

	return chk;
}

int
nmea_has_checksum(const char *sentence, size_t length, const char **chksum)
{
	const char *n = sentence + 1;
    int i=0;

	while ('*' != *n && NMEA_END_CHAR_1 != *n && '\0' != *n && ++i<length) {
		n++;
	}

    if ('*' == *n) {
        *chksum = n+1;
		return 0;
	}

	return -1;
}

int
nmea_validate(const char *sentence, size_t length, int check_checksum,
              char **sentence_end)
{
	const char *n;

	/* should have atleast 9 characters */
	if (9 > length) {
		return -1;
	}

	/* should start with $ */
	if ('$' != *sentence) {
		return -1;
	}

	/* should have a 5 letter, uppercase word */
	n = sentence;
	while (++n < sentence + 6) {
		if (*n < 'A' || *n > 'Z') {
			/* not uppercase letter */
			return -1;
		}
	}

	/* should have a comma after the type word */
	if (',' != sentence[6]) {
		return -1;
	}

	/* check for checksum */
    const char *sentence_actual_chksum=0;
	if (1 == check_checksum && 0 == nmea_has_checksum(sentence, length, &sentence_actual_chksum)) {
		uint8_t actual_chk;
		uint8_t expected_chk;
		char checksum[3];

        if (sentence_end) {
            *sentence_end = (char*)sentence_actual_chksum + 2;
        }
        
		checksum[0] = sentence_actual_chksum[0];
		checksum[1] = sentence_actual_chksum[1];
		checksum[2] = '\0';
		actual_chk = nmea_get_checksum(sentence);
		expected_chk = (uint8_t) strtol(checksum, NULL, 16);
		if (expected_chk != actual_chk) {
			return -1;
		}
	}

	return 0;
}

static const char *
parse_number(const char *str, int digits, int *val)
{
    const char *cp;
    const char *end;

    *val = 0;
    cp = str;
    end = str + digits;
    while (cp < end) {
        if (!isdigit((int) *cp)) {
            return (NULL);
        }
        *val *= 10;
        *val += (*cp - '0');
        cp++;
    }
    return (end);
}

static int
nmea_parse_time(const char *input, struct nmea_time *tm) {
    const char *cp = input;
    
    cp = parse_number(cp, 2, &tm->hour);
    if (cp == NULL) goto err;
    cp = parse_number(cp, 2, &tm->min);
    if (cp == NULL) goto err;
    cp = parse_number(cp, 2, &tm->sec);
    if (cp == NULL) goto err;

    tm->updated_at_usec = os_get_uptime_usec();
    return 0;
err:
    return 1;
}

static int
nmea_parse_date(const char *input, struct nmea_time *tm) {
    const char *cp = input;
    
    cp = parse_number(cp, 2, &tm->mday);
    if (cp == NULL) goto err;
    cp = parse_number(cp, 2, &tm->mon);
    if (cp == NULL) goto err;
    cp = parse_number(cp, 2, &tm->year);
    if (cp == NULL) goto err;

    if (tm->year < 100) tm->year += 2000;

    tm->updated_at_usec = os_get_uptime_usec();
    return 0;
err:
    return 1;
}

static int
nmea_parse_num(const char *input, int *value) {
    const char *cp = input;
    
    cp = parse_number(cp, strlen(input), value);
    if (cp == NULL) goto err;

    return 0;
err:
    return 1;
}

static int
nmea_parse_float(const char *input, float *value) {
    const char *cp = input;

	if (NULL == (cp = strchr(input, '.'))) {
        int v;
        int r = nmea_parse_num(input,&v);
        *value = (int)v;
		return r;
	}

    int whole_num;
    parse_number(input, cp-input, &whole_num);

    cp++; // Skip . 
    int n_decimals = 0;
    int divider = 1;
    for (int i=0;i<8;i++)
    {
        if (!isdigit(*(cp+i))) break;
        n_decimals++;
        divider*=10;
    }
    int decimals = 0;
    cp = parse_number(cp, n_decimals, &decimals);
    if (cp == NULL) goto err;
    *value = (float)whole_num + ((float)decimals)/divider;
    //printf("parse_float: '%s' whl=%d ndec:%d div:%d dec:%d\n", input, whole_num, n_decimals, divider, decimals);
    
    return 0;
err:
    return 1;
}

static int
nmea_parse_position(const char *input, struct nmea_position *pos) {
    const char *cp = input;
    const char *ep;

    pos->degrees = 0;
    pos->minutes = 0;
    
	/* decimal minutes */
	if (NULL == (cp = strchr(input, '.'))) {
		goto err;
	}

	/* minutes starts 2 digits before dot */
	cp -= 2;
    int minutes;
    cp = parse_number(cp, 2, &minutes);
    if (cp == NULL) goto err;
    cp++; // Skip . 
    int n_decimals = 0;
    int divider = 1;
    for (int i=0;i<8;i++)
    {
        if (!isdigit(*(cp+i))) break;
        n_decimals++;
        divider*=10;
    }
    int decimals = 0;
    cp = parse_number(cp, n_decimals, &decimals);
    pos->minutes = (double)minutes + ((double)decimals)/divider;

    //printf("parse_pos: '%s' min=%d ndec:%d div:%d dec:%d\n", input, minutes, n_decimals, divider, decimals);
    
	/* integer degrees */
    ep = strchr(input, '.');
    cp = parse_number(input, ep-input-2, &pos->degrees);

    return 0;
err:
    return 1;
}


nmea_t
nmea_get_type(const char *sentence)
{
    const char *s = sentence+1;
    //printf("get_type:%c%c%c%c%c\n",s[0],s[1],s[2],s[3],s[4]);
	if (!strncmp(s,"GPGGA",5) || !strncmp(s,"GNGGA",5)) {
		return NMEA_GPGGA;
	}
	if (!strncmp(s,"GPGLL",5) || !strncmp(s,"GNGLL",5)) {
		return NMEA_GPGLL;
	}
	if (!strncmp(s,"GPRMC",5) || !strncmp(s,"GNRMC",5)) {
		return NMEA_GPRMC;
	}
	if (!strncmp(s,"GPGSA",5) || !strncmp(s,"GNGSA",5)) {
		return NMEA_GPGSA;
	}
	if (!strncmp(s,"GPGSV",5) || !strncmp(s,"GNGSV",5)) {
		return NMEA_GPGSV;
	}
    return NMEA_UNKNOWN;
}

char *
nmea_parse(char *sentence, size_t length, int check_checksum,
           struct nmea *ns)
{
	unsigned int n_vals, val_index;
	char *value, *val_string;
	char *values[82];
    char *sentance_end = 0;
	nmea_t type;
    
	/* Validate sentence string */
	if (-1 == nmea_validate(sentence, length, check_checksum,
                            &sentance_end)) {
		return sentence;
	}

	type = nmea_get_type(sentence);    
	if (NMEA_UNKNOWN == type) {
		return sentence;
	}

	/* Crop sentence from type word */
	val_string = _crop_sentence(sentence, length);
	if (NULL == val_string) {
        return sentence;
	}

	/* Split the sentence into values */
	n_vals = _split_string_by_comma(val_string, values, ARRAY_LENGTH(values));
	if (0 == n_vals) {
		return sentence;
	}

	for (val_index = 0; val_index < n_vals; val_index++) {
		value = values[val_index];
		if (-1 == _is_value_set(value)) {
			continue;
		}

        switch (type)
        {
        case NMEA_GPGGA:
            if (val_index == NMEA_GPGGA_LATITUDE)
            {
                ns->errors += nmea_parse_position(value, &ns->latitude);
            }
            else if (val_index == NMEA_GPGGA_LATITUDE_CARDINAL)
            {
                ns->latitude.cardinal = nmea_cardinal_direction_parse(value);
                ns->latitude.decimal_deg = ns->latitude.degrees + ns->latitude.minutes/60.0;
                if (ns->latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH)
                    ns->latitude.decimal_deg = -ns->latitude.decimal_deg;
                ns->latitude.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPGGA_LONGITUDE)
            {
                ns->errors += nmea_parse_position(value, &ns->longitude);
            }
            else if (val_index == NMEA_GPGGA_LONGITUDE_CARDINAL)
            {
                ns->longitude.cardinal = nmea_cardinal_direction_parse(value);
                ns->longitude.decimal_deg = ns->longitude.degrees + ns->longitude.minutes/60.0;
                if (ns->longitude.cardinal == NMEA_CARDINAL_DIR_WEST)
                    ns->longitude.decimal_deg = -ns->longitude.decimal_deg;
                ns->longitude.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPGGA_TIME)
            {
                ns->errors += nmea_parse_time(value, &ns->time);
            }
            if (val_index == NMEA_GPGGA_N_SATELLITES)
            {
                ns->errors += nmea_parse_num(value, &ns->stats.n_satellites);
                ns->stats.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPGGA_ALTITUDE)
            {
                ns->errors += nmea_parse_float(value, &ns->altitude.altitude);
                ns->altitude.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPGGA_ALTITUDE_UNIT)
            {
                ns->altitude.unit = *value;
            }
            break;
        case NMEA_GPGLL:
            if (val_index == NMEA_GPGLL_LATITUDE)
            {
                ns->errors += nmea_parse_position(value, &ns->latitude);
            }
            else if (val_index == NMEA_GPGLL_LATITUDE_CARDINAL)
            {
                ns->latitude.cardinal = nmea_cardinal_direction_parse(value);
                ns->latitude.decimal_deg = ns->latitude.degrees + ns->latitude.minutes/60.0;
                if (ns->latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH)
                    ns->latitude.decimal_deg = -ns->latitude.decimal_deg;
                ns->latitude.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPGLL_LONGITUDE)
            {
                ns->errors += nmea_parse_position(value, &ns->longitude);
            }
            else if (val_index == NMEA_GPGLL_LONGITUDE_CARDINAL)
            {
                ns->longitude.cardinal = nmea_cardinal_direction_parse(value);
                ns->longitude.decimal_deg = ns->longitude.degrees + ns->longitude.minutes/60.0;
                if (ns->longitude.cardinal == NMEA_CARDINAL_DIR_WEST)
                    ns->longitude.decimal_deg = -ns->longitude.decimal_deg;
                ns->longitude.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPGLL_TIME)
            {
                ns->errors += nmea_parse_time(value, &ns->time);
            }
            break;
        case NMEA_GPRMC:
            if (val_index == NMEA_GPRMC_LATITUDE)
            {
                ns->errors += nmea_parse_position(value, &ns->latitude);
            }
            else if (val_index == NMEA_GPRMC_LATITUDE_CARDINAL)
            {
                ns->latitude.cardinal = nmea_cardinal_direction_parse(value);
                ns->latitude.decimal_deg = ns->latitude.degrees + ns->latitude.minutes/60.0;
                if (ns->latitude.cardinal == NMEA_CARDINAL_DIR_SOUTH)
                    ns->latitude.decimal_deg = -ns->latitude.decimal_deg;
                ns->latitude.updated_at_usec = os_get_uptime_usec();
            }
            if (val_index == NMEA_GPRMC_LONGITUDE)
            {
                ns->errors += nmea_parse_position(value, &ns->longitude);
                ns->longitude.decimal_deg = ns->longitude.degrees + ns->longitude.minutes/60.0;
                if (ns->longitude.cardinal == NMEA_CARDINAL_DIR_WEST)
                    ns->longitude.decimal_deg = -ns->longitude.decimal_deg;
                ns->longitude.updated_at_usec = os_get_uptime_usec();
            }
            else if (val_index == NMEA_GPRMC_LONGITUDE_CARDINAL)
            {
                ns->longitude.cardinal = nmea_cardinal_direction_parse(value);
            }
            if (val_index == NMEA_GPRMC_TIME)
            {
                ns->errors += nmea_parse_time(value, &ns->time);
            }
            if (val_index == NMEA_GPRMC_DATE)
            {
                ns->errors += nmea_parse_date(value, &ns->time);
            }
            break;
        default:
            break;
        }
    
	}

    if (sentance_end) {
        return sentance_end;
    } else {
        return values[n_vals - 1];
    }
}

int
nmea_has_valid_location(struct nmea *ns)
{
    if (fabs(ns->latitude.decimal_deg)  > 90.0 ||
        fabs(ns->longitude.decimal_deg) > 180.0) {
        return false;
    }
    
    if (fabs(ns->latitude.decimal_deg) > 0.00001 &&
        fabs(ns->longitude.decimal_deg) > 0.00001) {
        return true;
    }
    return false;
}
