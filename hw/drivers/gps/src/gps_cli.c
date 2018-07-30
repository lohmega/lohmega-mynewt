#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "defs/error.h"
#include <shell/shell.h>
#include <console/console.h>

#include "gps/gps.h"
#include "datetime/datetime.h"

#if MYNEWT_VAL(GPS_CLI)

static int gps_cli_cmd(int argc, char **argv);

#if MYNEWT_VAL(SHELL_CMD_HELP)
const struct shell_param cmd_gps_param[] = {
    {"status", "<none>"},
    {"setsystime", "[set system time from gps]"},
    {NULL,NULL},
};

const struct shell_cmd_help cmd_gps_help = {
	"gps commands", "<status>|<setsystime>", cmd_gps_param
};
#endif

static struct shell_cmd shell_gps_cmd = {
    .sc_cmd = "gps",
    .sc_cmd_func = gps_cli_cmd,
#if MYNEWT_VAL(SHELL_CMD_HELP)
    .help = &cmd_gps_help
#endif
};

static void
gps_cli_print_status(struct gps_dev* dev)
{
    int nmea_updates;
    gps_update(dev, &nmea_updates);
    struct nmea *ns = &(dev->ns);
    
    if (ns->time.updated_at_usec)
    {
        console_printf("NMEA DateTime: %04d-%02d-%02dT", ns->time.year, ns->time.mon, ns->time.mday);
        printf("%02d:%02d:%02d (%llds old)\n", ns->time.hour, ns->time.min, ns->time.sec,
               (os_get_uptime_usec() - ns->time.updated_at_usec)/1000000);
    }

    if (ns->latitude.updated_at_usec)
    {
        console_printf("NMEA Lat/Long: %d.%d", (int)ns->latitude.decimal_deg, abs((int)((ns->latitude.decimal_deg - (int)ns->latitude.decimal_deg)*1000000)));
        printf(",%d.%d", (int)ns->longitude.decimal_deg, abs((int)((ns->longitude.decimal_deg - (int)ns->longitude.decimal_deg)*1000000)));
        printf(" (%llds old)\n", (os_get_uptime_usec() - ns->latitude.updated_at_usec)/1000000);
    }
    if (ns->altitude.updated_at_usec)
    {
        console_printf("NMEA Altitude: %d.%d%c\n", (int)ns->altitude.altitude, (int)((ns->altitude.altitude-(int)ns->altitude.altitude)*100), (char)ns->altitude.unit);
    }
    if (ns->altitude.updated_at_usec || 1)
    {
        console_printf("NMEA Stats:    %d satellites\n", ns->stats.n_satellites);
    }

}


static int
gps_cli_cmd(int argc, char **argv)
{
    struct gps_dev *dev;
    struct os_timeval tv;
    struct os_timezone tz;
    struct clocktime ct;
        
    dev = (struct gps_dev *) os_dev_open("gps0", OS_TIMEOUT_NEVER, NULL);

    if (argc < 2) {
        gps_cli_print_status(dev);
        return 0;
    }

    if (!strcmp(argv[1], "status")) {
        gps_cli_print_status(dev);
    } else if (!strcmp(argv[1], "setsystime")) {
        if (!gps_set_systime(dev))
        {
            os_gettimeofday(&tv, &tz);
            timeval_to_clocktime(&tv,&tz,&ct);
            console_printf("Time updated from GPS to: %04d-%02d-%02dT", ct.year, ct.mon, ct.day);
            console_printf("%02d:%02d:%02d.%03d\n", ct.hour, ct.min, ct.sec,ct.usec/1000);
        }
    } else {
        console_printf("Unknown cmd\n");
    }

    os_dev_close((struct os_dev*) dev);
    return 0;
}


int
gps_cli_register(void)
{
    return shell_cmd_register(&shell_gps_cmd);
}

#endif
