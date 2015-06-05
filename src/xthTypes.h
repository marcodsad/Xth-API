//////////////////////////////////////
//  xthTypes.h
//
//
//


#include <iostream>
#define LOGTEN 2.302585092994

using namespace std;


// serial includes
#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h> /* for ioctl DTR */
#include <termios.h> /* for TERMIO ioctl calls */
#include <unistd.h>
#include <glob.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>


// using t_symbol from m_pd.h
typedef struct _xthSymbol
{
    char *s_name;
    struct _class **s_thing;
    struct _xthSymbol *s_next;
} t_xthSymbol;

typedef struct{
    long            n; /* the state of a last input */

    int             comhandle; /* holds the komport handle */
    struct termios  oldcom_termio; /* save the old com config */
    struct termios  com_termio; /* for the new com config */

    t_xthSymbol        *serial_device;
    char            serial_device_prefix[FILENAME_MAX];/* the device name without the number */
    short           komport; /* holds the komport # */
    float         baud; /* holds the current baud rate */
    float         data_bits; /* holds the current number of data bits */
    float         parity_bit; /* holds the current parity */
    float         stop_bits; /* holds the current number of stop bits */
    int             xonxoff; /* nonzero if xonxoff handshaking is on */
    int             ctsrts; /* nonzero if ctsrts handshaking is on */
    int             hupcl; /* nonzero if hang-up on close is on */
    short           rxerrors; /* holds the rx line errors */
    int             x_hit;
    double          x_deltime;
    int             verbose;
    unsigned char   *x_inbuf; /* read incoming serial to here */
    unsigned char   *x_outbuf; /* write outgoing serial from here */
    int             x_inbuf_len; /* length of inbuf */
    int             x_outbuf_len; /* length of outbuf */
    int             x_outbuf_wr_index; /* offset to next free location in x_outbuf */
} xthSerial;
// end of serial definitions






typedef struct{
    float raw;
    float nat;
    float soft;
    float lin;
    float tanh;
    float max;
} xthMmg;

typedef struct{
    float x;
    float y;
    float z;
} xthAccel;

//  configuration of the XS
typedef struct{
    int samplerate;
    bool ledActive;
    bool mmgActive;
    bool tempActive;
    bool accelActive;
    bool gyroActive;
    bool magnetActive;

} xthConfig;

//  parameters of the XS system
typedef struct{
    int testElement;
} xthParameters;

//  dataspace of the XS system
typedef struct{
	xthMmg mmgVals;
	float tempVal;
	xthAccel accelVals;
    float flowAscendentStorage;
    float flowAverageStorage[2];
    float flowBetweenBounds[2];
    float flowLastMin;
    float flowLastMax;
    float curveDrawCurve[100];
    float smoothFactor;
	float smoothPrevIn;
	float smoothPrevOut;
} xthDataspace;
