
/* default settings of the HC-12 transmitter device
 *  - baudrate =  9600 bps
 *    "AT+Bxxxx" , xxxx = 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
 *  - channel  = 1
 *    "AT+Cxxx" , xxx = 1 ... 100 (1...5 allowed in CET)
 *  - power    = 5
 *    "AT+Px"    , x = 1 ... 8 (1...4 (5) allowed in CET)
 *                 1   2   3   4   5   6   7   8    value
 *                --------------------------------+-----------
 *                -1   2   5   8  11   14  17  20  power (dBm)
 *  - com mode = 3
 *    "AT+FUx"   , x = 1 ... 4
 *  other commands:
 *   "AT"         -- response test
 *   "AT+RX"      -- enter receive mode (default)
 *   "AT+VERSION" -- print FW version number
 *   "AT+SLEEP"   -- enter sleep mode
 *   "AT+DEFAULT" -- restore factory defaults
 *   "AT+UPDATE"  -- firmware update (!)
 *
 * serial pin   PI Z-II  Odroid C5  Odroid M1 Radxa C4+ |
 * -----------+---------+----------+---------+----------+
 *    S0.Tx   |    8    |    8     |   13    |    8     | GPIO-Pin
 *    S0.Rx   |   10    |   10     |   11    |   10     | GPIO-Pin
 *    -SET-   | 17 (11) | 28  (7)  | 7  (7)  | 131 (12) | Line (GPIO-Pin)
 */

#define _SINGLE_CHAR_RECEPTION_

#define TIMEOUT             60

/* line number evaluated from a 'gpioinfo' call on the target
 */
#ifdef _ODROID_C5_
  #define LINE_SET          28             /* C5  --> GPIOD_4 (Pin 7) = line 28 */
  #define GPIO_CHIP         "gpiochip0"    /* check with target platform ! */
#endif
#ifdef _ODROID_M1_
  #define LINE_SET          7              /* M1  --> GPIO_14 (Pin 7) = line 7 */
  #define GPIO_CHIP         "gpiochip0"    /* check with target platform ! */
#endif
#ifdef _RADXA_C4P_
  #define LINE_SET         131             /* C4+  --> GPIO_131 (Pin 12) = line 131 */
  #define GPIO_CHIP         "gpiochip0"    /* check with target platform ! */
#endif
#ifdef _PI_ZERO_2_
  #define LINE_SET          17             /* RPI --> GPIO_17 (Pin 11) */
  #define GPIO_CHIP         "gpiochip0"
#endif
#ifdef _PC_HOST_
  #define LINE_SET          0              /* unused for the host build */
  #define GPIO_CHIP         "gpiochip0"    /* irrelevant here */
#endif

#ifdef _PC_HOST_
  #define SERPORT_HC12        "/dev/ttyUSB0"
#else
  #define SERPORT_HC12        "/dev/ttyS0"
#endif
#define SERLINE_HC12        0
#define BAUDRATE_HC12       B9600
#define SET_POWER           "AT+P"
#define SET_BAUDRATE        "AT+B"
#define SET_CHANNEL         "AT+C"
#define SET_MODE            "AT+FU"
#define DEFAULT_PWR_LEVEL   4
#define DEFAULT_BAUDRATE    9600
#define DEFAULT_CHANNEL     1
#define DEFAULT_MODE        3

#define  MAX_PO_LEVEL       8
#define  MAX_CHANNEL        100
#define  MAX_FU_MODE        4

#define MODE_RX             0  /* application mode Rx - default */
#define MODE_TX             1  /* application mode tx */

/* ---- data types ----
 */
struct hc12Setup
{
    unsigned  baudrate;        /* baud rate, <1200..115200> */
    unsigned  power;           /* power level, <1..8>       */
    unsigned  channel;         /* channel, <1..100>         */
    unsigned  fu_mode;         /* fu mode, <1..4>           */
    unsigned  trx_mode;        /* device mode (default: Rx) */
    char     *pSerLine;        /* serial line name (ttyS0)  */
};

static char at_test[]     = "AT\n";
static char at_rx[]       = "AT+RX\n";
static char at_version[]  = "AT+VERSION\n";
static char at_sleep[]    = "AT+SLEEP\n";
static char at_default[]  = "AT+DEFAULT\n";
static char at_update[]   = "AT+UPDATE\n";

static char testmsg []    = "test message from HC12 transmitter !\r\n";

static char at_cmd [16]   = {'\0'};
static char rxBuffer[256] = {'\0'};

