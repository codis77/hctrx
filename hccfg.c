/* application to set the HC12 transmitter parameters via serial line
 *  --- f.m.  02.09.2025
 * 
 * the HC12 transmitter stores the interface and transmission parameters in non-volatile
 * memory (baudrate, power, channel, FU mode); thus this settings are persistent;
 * however, they are only valid in operational mode;
 * configuration mode (SET pin is LOW) always uses 9600 bps@ 8N1 !
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <signal.h>
#include <gpiod.h>

#include "hctrx.h"


static const char         *chipname   = GPIO_CHIP;
static struct gpiod_chip  *chip;
static struct gpiod_line  *lineSET = NULL;  /* SET line */

static int                 cfgBaudrate = B9600;
static int                 termDelay = 2;
static struct termios      options, oldoptions;

static int                 stopRx = 0;
static int                 verbosemode = 0;

static char                optstr[] = {":hvp:c:b:f:t"};  // initial ':' to suppress internal error messages

/* -- prototypes --
 */
static int     openLine          (char *szPort, speed_t BaudRate, int timeout, struct termios *poptions);
static void    hc12_pinSetup     (void);
static int     getTrxCmdResponse (int fh, char *buffer);
static int     getMode           (char *argstr);
static void    inthandler        (int d);
static int     waitRx            (int iFDtrx, char *rxBuffer);
static int     getArgs           (int argc, char **argv, struct hc12Setup *pSetup);
static speed_t getBRIndex        (int brValue);
static void    printHelp         (char *appName);
static void    setdefaults       (struct hc12Setup *pSetup);

/* -- code --
 */
int  main (int argc, char *argv[])
{
    struct hc12Setup  hcsetup;
    int               i, iFDtrx, count, mode, txlen;
    char             *p, *pSerLine, value, c;
    char             *txMsg;
    speed_t           BaudRate;

    /* get arguments */
    setdefaults (&hcsetup);
    i = getArgs (argc, argv, &hcsetup);

    hc12_pinSetup();

    /* open the serial port */
    /* might be incorrect, perhaps the config interface needs always be opened
     * with 9600bps here, and the baudrate is only for Tx/Rx mode */
    if ((iFDtrx = openLine (hcsetup.pSerLine, cfgBaudrate, termDelay, &oldoptions)) == -1)
    {
        fprintf (stderr, "error opening %s: %s !\n", SERPORT_HC12, strerror(errno));
        return 10;
    }

printf ("opened ser.line %s, (fh=%d,bri=%d\n", pSerLine, iFDtrx, BaudRate);

    /* set to config mode */
#ifndef _PC_HOST_
    gpiod_line_set_value (lineSET, 0);       /* low-active ! */
#endif

    /* wait 200ms before trying to send commands*/
    usleep (200000);

    /* check response */
    p = at_test;
    i = write (iFDtrx, p, strlen(p));
    if (count <= 0)
    {
        printf ("error: no response !\n");
        goto error;
    }

    /* set Tx power */
    fflush (stdout);
    sprintf (at_cmd, "%s%d\n", SET_POWER, hcsetup.power);
    (void) write (iFDtrx, at_cmd, strlen(p));
//    usleep (20000);
    count = getTrxCmdResponse (iFDtrx, rxBuffer);
    usleep (20000);

    /* set channel */
    sprintf (at_cmd, "%s%03d\n", SET_CHANNEL, hcsetup.channel);
    printf ("%s", at_cmd);
    fflush (stdout);
    (void) write (iFDtrx, at_cmd, strlen(p));
    count = getTrxCmdResponse (iFDtrx, rxBuffer);
    usleep (20000);

    /* set mode */
    sprintf (at_cmd, "%s%d\n", SET_MODE, hcsetup.fu_mode);
    printf ("%s", at_cmd);
    fflush (stdout);
    (void) write (iFDtrx, at_cmd, strlen(p));
    count = getTrxCmdResponse (iFDtrx, rxBuffer);
    usleep (20000);

    fflush (stdout);
    sprintf (at_cmd, "%s%d\n", SET_BAUDRATE, hcsetup.baudrate);
    (void) write (iFDtrx, at_cmd, strlen(p));
//    usleep (20000);
    count = getTrxCmdResponse (iFDtrx, rxBuffer);
    usleep (20000);

#if 0
  #ifdef _PC_HOST_
    printf ("\n -- initialisations done, remove SET jumper now to receive/transmit data !\n");
    printf (" press <CR> to continue : ");
    fflush (stdout);
    fflush (stdin);
    do
        c = getchar ();
    while (c != '\n');
  #endif
#endif

    /* un-set config mode, and wait a moment before terminating */
#ifndef _PC_HOST_
    gpiod_line_set_value (lineSET, 1);
#endif
    usleep (200000);

error:               /* jump label for errors */
printf ("terminating...\n");
    fflush (stdout);
    close(iFDtrx);
    return EXIT_SUCCESS;
}



// open the serial interface
// takes the serial port ("/dev/ttyS<n>"), baudrate index,
// and inter-character timeout as parameters;
// returns the file handle for the opened device
int  openLine (char *szPort, speed_t BaudRate, int timeout, struct termios *poptions)
{
    int  iFD;

    iFD = open (szPort, O_RDWR | O_NOCTTY); // | O_NDELAY);

    // configure port reading; block read(), until data arrive
    fcntl (iFD, F_SETFL, 0);

    // Get the current options for the port
    tcgetattr (iFD, poptions);
    memcpy (&options, poptions, sizeof (struct termios));

    // Set the baud rates to the defined value
    cfsetispeed (&options, BaudRate);
    cfsetospeed (&options, BaudRate);

    // Enable the receiver and set local mode
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;                 // Mask the character size to 8 bits, no parity
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |=  CS8;                    // Select 8 data bits
    options.c_cflag &= ~CRTSCTS;                // Disable hardware flow control

    // Enable data to be processed as raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(ICRNL | INLCR | IXON | IXOFF);
    options.c_oflag &= ~OPOST;

    options.c_cc[VTIME] = 3;//timeout;              // inter-character timer, in 10th of a second

//  options.c_cc[VMIN]  = 1;                    // # of characters expected at least
    options.c_cc[VMIN]  = 0;                    // block read, hard timeout

    // options.c_cc[VEOF] = 4;

    // enable the new options for the port
    tcsetattr (iFD, TCSANOW, &options);

    return (iFD);
}



static void inthandler (int dummy)
{
    stopRx = 1;
}


#ifdef _PC_HOST_
static void  hc12_pinSetup (void)
{
    return;
}
#else  // Raspberry PI
static void  hc12_pinSetup (void)
{
    int  i;

    chip    = gpiod_chip_open_by_name (chipname);
    lineSET = gpiod_chip_get_line (chip, LINE_SET);
    if ((i = gpiod_line_request_output (lineSET, "hc12_set", 1)) != 0)
        printf ("error requesting SET GPIO line (%d) !\n", i);
    return;
}

#endif


static int  getTrxCmdResponse (int fh, char *buffer)
{
    int    i, c, ix;
    char   lbuf[32];
    char  *p0;

    ix = 0;
    *buffer = '\0';
    p0 = buffer;
    do
    {
        c = lbuf[0] = 0;
        if ((i = read (fh, lbuf, 1)) > 0)
        {
            int  k;
            for (k=0; k<i; k++)
            {
                *buffer++ = lbuf[k];
                c = lbuf[k];
//              printf ("%02x ", c); fflush (stdout);
                ix++;
            }
        }
    }
    while (c != '\n');

    *buffer = '\0';
//  printf ("getResp -> ret = %d, buf=%s", ix, p0);
    fflush (stdout);
    return (ix);
}



static int  waitRx (int fh, char *buffer)
{
    char  lbuf[16];
    int   i, k, ix;

    ix = 0;
    do
    {
        if ((i = read (fh, lbuf, 1)) > 0)
        {
            int  k;
            for (k=0; k<i; k++)
            {
                *buffer++ = lbuf[k];
//              printf ("%02x ", c); fflush (stdout);
                ix++;
            }
        }
    }
    while (i > 0);

    return ix;
}



static int  getArgs (int argc, char **argv, struct hc12Setup *pSetup)
{
    struct tm  *ltm;
    time_t      t;
    int         k, doHelp, doExit, args;
    char        c;

    if (argc == 1)
        return 0;

    doHelp = doExit = args = 0;

    unsigned  baudrate;        /* baud rate, <1200..115200> */

    while ((c = getopt (argc, argv, optstr)) != -1)
    {
        switch (c)
        {
            case 'p':    // output power level
                k = (int) strtol (optarg, NULL, 10);
                if ((k>0) && (k <= MAX_PO_LEVEL))
                    pSetup->power = k;
                args++;
            break;
            case 'c':    // channel number
                k = (int) strtol (optarg, NULL, 10);
                if ((k>0) && (k <= MAX_CHANNEL))
                    pSetup->channel = k;
                args++;
                break;
            case 'b':    // baud rate
                k = (int) strtol (optarg, NULL, 10);
                pSetup->baudrate = getBRIndex (k);
                args++;
                break;
            case 'f':    // fu - mode
                k = (int) strtol (optarg, NULL, 10);
                if ((k>0) && (k <= MAX_FU_MODE))
                    pSetup->fu_mode = k;
                args++;
                break;
            case 't':    // transmit mode
                pSetup->trx_mode = MODE_TX;
                args++;
                break;
            case '?':
                if ((strchr (optstr, c)) && (optopt != 'h'))
                    fprintf (stderr, "Option -%c requires an argument.\n", optopt);
                else
                    fprintf (stderr, "Unknown option `-%c'.\n", optopt);
            case 'h':    // fall-through, output help screen
                doHelp = 1;
                break;
            case 'v':
                verbosemode = 1;
                break;
            default:
                doExit = 1;
        }
        if (doHelp | doExit)
            break;
    }
    /* a serial interface name is expected as non-option parameter */
    if ((c == -1) && (optind <= argc))
    {
        pSetup->pSerLine = argv[optind];
        args++;
    }

    if (doHelp)
        printHelp (argv[0]);

    if (doHelp | doExit)
        return (-10);
    else
        return (args);
}

/* translate a baudrate value into the corresponding index;
 * returns a speed_t buadrate index;
 * the value is B9600 for invalid baudrates
 */
static speed_t  getBRIndex (int brValue)
{
    if (brValue == 1200)
        return B1200;
    if (brValue == 2400)
        return B2400;
    if (brValue == 4800)
        return B4800;
    if (brValue == 9600)
        return B9600;
    if (brValue == 19200)
        return B19200;
    if (brValue == 38400)
        return B38400;
    if (brValue == 57600)
        return B57600;
    if (brValue == 115200)
        return B115200;

    /* default value */
    return B9600;
}



static void  printHelp (char *appName)
{
    fprintf (stdout, "  hccfg - a tool to runtime-configure a HC12 transmitter\n\n");
    fprintf (stdout, "call as : hccfg [options] <serial_line>\n");
    fprintf (stdout, "recognized options are :\n");
    fprintf (stdout, "    -b <baudrate> = serial baudrate used\n");
    fprintf (stdout, "                    standard baudrates from 1200 to 115200 are supported\n");
    fprintf (stdout, "    -c <channel>  = channel number (1..%d)\n", MAX_CHANNEL);
    fprintf (stdout, "    -p <power>    = output power level (1..%d)\n", MAX_PO_LEVEL);
    fprintf (stdout, "    -f <FU mode>  = FU mode (1..%d)\n", MAX_FU_MODE);
    fprintf (stdout, " <serial_line> is the serial interface, e.g. ttyS0, ttyUSB0\n");
    fprintf (stdout, "for options not given, default parameters are used\n");
    fprintf (stdout, "(9600bps, Ch1, PowerLevel 4, FU mode = 3, %s)\n", SERPORT_HC12);
}



static void  setdefaults (struct hc12Setup *pSetup)
{
    pSetup->baudrate = DEFAULT_BAUDRATE;
    pSetup->power    = DEFAULT_PWR_LEVEL;
    pSetup->channel  = DEFAULT_CHANNEL;
    pSetup->fu_mode  = DEFAULT_MODE;
    pSetup->trx_mode = 0;
    pSetup->pSerLine = SERPORT_HC12;
}
