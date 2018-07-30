#ifndef __NMEA_H__
#define __NMEA_H__

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* NMEA sentence types */
typedef enum {
	NMEA_UNKNOWN,
	NMEA_GPGGA,
	NMEA_GPGLL,
	NMEA_GPRMC,
    NMEA_GPGSA,
    NMEA_GPGSV,
} nmea_t;

/* NMEA cardinal direction types */
typedef char nmea_cardinal_t;
#define NMEA_CARDINAL_DIR_NORTH		(nmea_cardinal_t) 'N'
#define NMEA_CARDINAL_DIR_EAST		(nmea_cardinal_t) 'E'
#define NMEA_CARDINAL_DIR_SOUTH		(nmea_cardinal_t) 'S'
#define NMEA_CARDINAL_DIR_WEST		(nmea_cardinal_t) 'W'
#define NMEA_CARDINAL_DIR_UNKNOWN	(nmea_cardinal_t) '\0'

/* GPS position struct */
struct nmea_position{
	double minutes;
	int degrees;
	double decimal_deg;
	nmea_cardinal_t cardinal;
    int64_t updated_at_usec;
} ;

struct nmea_stats{
    int n_satellites;
    int64_t updated_at_usec;
};

struct nmea_altitude{
    float altitude;
    uint8_t unit;
    int64_t updated_at_usec;
};

struct nmea_time {
    int sec;
    int min;
    int hour;
    int mday;
    int mon;
    int year;
    int64_t updated_at_usec;
};

struct nmea {
	nmea_t type;
	int errors;
    struct nmea_altitude altitude;
    struct nmea_position latitude;
    struct nmea_position longitude;
    struct nmea_time time;
    struct nmea_stats stats;
};

/* NMEA sentence max length, including \r\n (chars) */
#define NMEA_MAX_LENGTH		82

/* NMEA sentence endings, should be \r\n according the NMEA 0183 standard */
#define NMEA_END_CHAR_1		'\n'
#define NMEA_END_CHAR_2		'\n'

/* NMEA sentence prefix length (num chars), Ex: GPGLL */
#define NMEA_PREFIX_LENGTH	5

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get the sentence type.
 *
 * sentence needs to be a validated NMEA sentence string.
 *
 * Returns nmea_t (int).
 */
nmea_t nmea_get_type(const char *sentence);

/**
 * Calculate the checksum of the sentence.
 *
 * sentence needs to be a validated NMEA sentence string.
 *
 * Returns the calculated checksum (uint8_t).
 */
uint8_t nmea_get_checksum(const char *sentence);

/**
 * Check if the sentence contains a precalculated checksum.
 *
 * sentence needs to be a validated NMEA sentence string.
 * length is the character length of the sentence string.
 *
 * Return 0 if checksum exists, otherwise -1.
 */
int nmea_has_checksum(const char *sentence, size_t length, const char **chksum);

/**
 * Validate the sentence according to NMEA 0183.
 *
 * Criterias:
 *   - Should be between the correct length.
 *   - Should start with a dollar sign.
 *   - The next five characters should be uppercase letters.
 *   - If it has a checksum, check it.
 *   - Ends with the correct 2 characters.
 *
 * length is the character length of the sentence string.
 *
 * Returns 0 if sentence is valid, otherwise -1.
 */
int nmea_validate(const char *sentence, size_t length, int check_checksum,
                  char **sentance_end);

/**
 * Has valid location
 *
 * data should be a pointer to a struct nmea.
 */
int nmea_has_valid_location(struct nmea *data);
    

/**
 * Parse an NMEA sentence string to a struct.
 *
 * sentence needs to be a validated NMEA sentence string.
 * length is the character length of the sentence string.
 * check_checksum, if 1 and there is a checksum, validate it.
 *
 * Returns a pointer to an NMEA data struct, or (nmea_s *) NULL if an error occurs.
 */
char *
nmea_parse(char *sentence, size_t length, int check_checksum, struct nmea *ns);

#ifdef __cplusplus
}
#endif


#endif
