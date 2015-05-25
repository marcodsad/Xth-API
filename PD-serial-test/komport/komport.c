/* komport - PD external for unix/windows

 (c) 1998-2005  Winfried Ritsch (see LICENCE.txt)
 Institute for Electronic Music - Graz

*/

#include "m_pd.h"



#include <sys/time.h>
#include <fcntl.h>
#include <sys/ioctl.h> /* for ioctl DTR */
#include <termios.h> /* for TERMIO ioctl calls */
#include <unistd.h>
#include <glob.h>
#define HANDLE int
#define INVALID_HANDLE_VALUE -1

#include <string.h>
#include <errno.h>
#include <stdio.h>


typedef struct komport
{
    t_object        x_obj;
    long            n; /* the state of a last input */

    int             comhandle; /* holds the komport handle */
    struct termios  oldcom_termio; /* save the old com config */
    struct termios  com_termio; /* for the new com config */

    t_symbol        *serial_device;
    char            serial_device_prefix[FILENAME_MAX];/* the device name without the number */
    short           komport; /* holds the komport # */
    t_float         baud; /* holds the current baud rate */
    t_float         data_bits; /* holds the current number of data bits */
    t_float         parity_bit; /* holds the current parity */
    t_float         stop_bits; /* holds the current number of stop bits */
    int             xonxoff; /* nonzero if xonxoff handshaking is on */
    int             ctsrts; /* nonzero if ctsrts handshaking is on */
    int             hupcl; /* nonzero if hang-up on close is on */
    short           rxerrors; /* holds the rx line errors */
    t_clock         *x_clock;
    int             x_hit;
    double          x_deltime;
    int             verbose;
    t_outlet        *x_data_outlet;
    t_outlet        *x_status_outlet;
    unsigned char   *x_inbuf; /* read incoming serial to here */
    unsigned char   *x_outbuf; /* write outgoing serial from here */
    int             x_inbuf_len; /* length of inbuf */
    int             x_outbuf_len; /* length of outbuf */
    int             x_outbuf_wr_index; /* offset to next free location in x_outbuf */
} t_komport;

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#ifndef ON
#define ON 1
#define OFF 0
#endif

/* Serial Port Return Values */

#define NODATAAVAIL -1
#define RXERRORS -2
#define RXBUFOVERRUN -4
#define TXBUFOVERRUN -5

#define komport_MAX 99
#define USE_DEVICENAME 9999 /* use the device name instead of the number */
#define komport_BUF_SIZE 16384 /* this should be the largest possible packet size for a USB com port */



#ifdef  IRIX
#define OPENPARAMS (O_RDWR|O_NDELAY|O_NOCTTY)
#define TIONREAD FIONREAD         /* re map the IOCTL function */
#define BAUDRATE_230400 -1
#define BAUDRATE_115200 -1
#define BAUDRATE_57600  -1
#define BAUDRATE_38400  B38400
#else /* IRIX */
#define OPENPARAMS (O_RDWR|O_NDELAY|O_NOCTTY)
#define BAUDRATE_230400 B230400
#define BAUDRATE_115200 B115200
#define BAUDRATE_57600  B57600
#define BAUDRATE_38400  B38400
#endif /* else IRIX */

static
long baudspeedbittable[] =
{
    BAUDRATE_230400,
    BAUDRATE_115200,  /* CPU SPECIFIC */
    BAUDRATE_57600,   /* CPU SPECIFIC */
    BAUDRATE_38400,   /* CPU SPECIFIC */
    B19200,
    B9600,
    B4800,
    B2400,
    B1800,
    B1200,
    B600,
    B300,
    B200,
    B150,
    B134,
    B110,
    B75,
    B50,
    B0
};

struct timeval null_tv;

#define BAUDRATETABLE_LEN 19

static long baudratetable[] =
{
    230400L,
    115200L,
    57600L,
    38400L,
    19200L,
    9600L,
    4800L,
    2400L,
    1800L,
    1200L,
    600L,
    300L,
    200L,
    150L,
    134L,
    110L,
    75L,
    50L,
    0L

}; /* holds the baud rate selections */


/* From man cfsetospeed:
       cfsetospeed()  sets  the  output  baud  rate stored in the
       termios structure pointed to by termios_p to speed,  which
       must be one of these constants:
            B0
            B50
            B75
            B110
            B134
            B150
            B200
            B300
            B600
            B1200
            B1800
            B2400
            B4800
            B9600
            B19200
            B38400
            B57600
            B115200
            B230400
  The  zero  baud rate, B0, is used to terminate the connec­
  tion.  If B0 is specified, the modem control  lines  shall
  no longer be asserted.  Normally, this will disconnect the
  line.*/

t_class *komport_class;

static void komport_pollintervall(t_komport *x, t_floatarg g);
static void komport_tick(t_komport *x);
static float set_baudrate(t_komport *x, t_float baud);
static float set_bits(t_komport *x, int nr);
static float set_parity(t_komport *x,int n);
static float set_stopflag(t_komport *x, t_float nr);
static int set_ctsrts(t_komport *x, int nr);
static int set_dtr(t_komport *x, int nr);
static int set_rts(t_komport *x, int nr);
static int set_xonxoff(t_komport *x, int nr);
static int set_serial(t_komport *x);
static int write_serial(t_komport *x, unsigned char serial_byte);
static int write_serials(t_komport *x, unsigned char *serial_buf, int buf_length);
static int komport_get_dsr(t_komport *x);
static int komport_get_cts(t_komport *x);

static int set_hupcl(t_komport *x, int nr);
static int open_serial(unsigned int com_num, t_komport *x);
static int close_serial(t_komport *x);
static long get_baud_ratebits(t_float *baud);

static void komport_pollintervall(t_komport *x, t_floatarg g);
static void komport_tick(t_komport *x);
static void komport_float(t_komport *x, t_float f);
static void komport_list(t_komport *x, t_symbol *s, int argc, t_atom *argv);
static void *komport_new(t_symbol *s, int argc, t_atom *argv);
static void komport_free(t_komport *x);
static void komport_baud(t_komport *x,t_floatarg f);
static void komport_bits(t_komport *x,t_floatarg f);
static void komport_parity(t_komport *x,t_floatarg f);
static void komport_stopbit(t_komport *x,t_floatarg f);
static void komport_rtscts(t_komport *x,t_floatarg f);
static void komport_dtr(t_komport *x,t_floatarg f);
static void komport_rts(t_komport *x,t_floatarg f);
static void komport_xonxoff(t_komport *x,t_floatarg f);
static void komport_hupcl(t_komport *x,t_floatarg f);
static void komport_close(t_komport *x);
static void komport_open(t_komport *x, t_floatarg f);
static void komport_devicename(t_komport *x, t_symbol *s);
static void komport_print(t_komport *x, t_symbol *s, int argc, t_atom *argv);
static void komport_output_status(t_komport *x, t_symbol *selector, t_float output_value);
static void komport_output_port_status(t_komport *x);
static void komport_output_dsr_status(t_komport *x);
static void komport_output_cts_status(t_komport *x);
static void komport_output_baud_rate(t_komport *x);
static void komport_output_parity_bit(t_komport *x);
static void komport_output_stop_bits(t_komport *x);
static void komport_output_data_bits(t_komport *x);
static void komport_output_rtscts(t_komport *x);
static void komport_output_xonxoff(t_komport *x);
static void komport_output_hupcl(t_komport *x);
static void komport_output_rxerrors(t_komport *x);
static void komport_enum(t_komport *x);
static void komport_info(t_komport *x);
static void komport_devices(t_komport *x);
static void komport_ports(t_komport *x);
static void komport_verbose(t_komport *x, t_floatarg f);
static void komport_help(t_komport *x);
void komport_setup(void);

/* --------- sys independent serial setup helpers ---------------- */



/* ------------ sys dependent serial setup helpers ---------------- */


/* ----------------- POSIX - UNIX ------------------------------ */


static long get_baud_ratebits(t_float *baud)
{
    int i = 0;

    while(i < BAUDRATETABLE_LEN && baudratetable[i] > *baud) i++;

    if(baudratetable[i] != *baud)
        post("[komport]: %d not valid, using closest value: ", *baud, baudratetable[i]);

    /* nearest Baudrate finding */
    if(i==BAUDRATETABLE_LEN ||  baudspeedbittable[i] < 0)
    {
        post("*Warning* The baud rate %d is not supported or out of range, using 9600\n",*baud);
        i = 8;
    }
    *baud =  baudratetable[i];
    post("get_baud_ratebits: %g", *baud);

    return baudspeedbittable[i];
}

static float set_baudrate(t_komport *x, t_float baud)
{
    struct termios  *tio = &(x->com_termio);
    speed_t         baudbits = get_baud_ratebits(&baud);
    post("set_baudrate baudbits: %d", baudbits);
    if( cfsetispeed(tio, baudbits) != 0 )
        post("[komport]: ERROR failed to set bitrate: %d", baudbits);
    if( cfsetospeed(tio, baudbits) != 0 )
        post("[komport]: ERROR failed to set bitrate: %d", baudbits);

    return baud;
}

/* bits are 5,6,7,8(default) */

static float set_bits(t_komport *x, int nr)
{
    struct termios *tio = &(x->com_termio);
    tio->c_cflag &= ~CSIZE;
    switch(nr)
    {
        case 5: tio->c_cflag |= CS5; return 5;
        case 6: tio->c_cflag |= CS6; return 6;
        case 7: tio->c_cflag |= CS7; return 7;
        default: tio->c_cflag |= CS8;
    }
    return 8;
}


/* 1 ... Parity even, -1 parity odd , 0 (default) no parity */
static float set_parity(t_komport *x,int n)
{
    struct termios *tio = &(x->com_termio);

    switch(n)
    {
        case 1:
            tio->c_cflag |= PARENB;  tio->c_cflag &= ~PARODD; return 1;
        case -1:
            tio->c_cflag |= PARENB | PARODD; return -1;
        default:
            tio->c_cflag &= ~PARENB;
    }
    return 0;
}


/* activate second stop bit with 1, 0(default)*/
static float set_stopflag(t_komport *x, t_float f_nr)
{
    int nr = (int)f_nr;
    struct termios *tio = &(x->com_termio);

    if(nr == 1)
    {
        tio->c_cflag |= CSTOPB;
        return 1;
    }
    else tio->c_cflag &= ~CSTOPB;

    return 0;
}

/* never tested */
static int set_ctsrts(t_komport *x, int nr)
{
    struct termios *tio = &(x->com_termio);

    if(nr == 1)
    {
        tio->c_cflag |= CRTSCTS;
        return 1;
    }
    tio->c_cflag &= ~CRTSCTS;
    return 0;
}

static int set_dtr(t_komport *x, int nr)
{
    int fd = x->comhandle;
    int status;

    if (fd == INVALID_HANDLE_VALUE) return -1;

    ioctl(fd, TIOCMGET, &status);
     if (nr == 0)
        status &= ~TIOCM_DTR;
    else
        status |= TIOCM_DTR;
    ioctl(fd, TIOCMSET, &status);
    return (nr !=0);
}

static int set_rts(t_komport *x, int nr)
{
    int fd = x->comhandle;
    int status;

    if (fd == INVALID_HANDLE_VALUE) return -1;

    ioctl(fd, TIOCMGET, &status);
    if (nr == 0)
        status &= ~TIOCM_RTS;
    else
        status |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    return (nr !=0);
}

static int set_xonxoff(t_komport *x, int nr)
{
    struct termios *tio = &(x->com_termio);

    if(nr == 1)
    {
        tio->c_iflag |= (IXON | IXOFF | IXANY);
        return 1;
    }

    tio->c_iflag &= ~IXON & ~IXOFF &  ~IXANY;
    return 0;
}

static int set_hupcl(t_komport *x, int nr)
{
    struct termios  settings;
    int             result;

    result = tcgetattr(x->comhandle, &settings);
    if (result < 0)
    {
        perror ("error in tcgetattr");
        return 0;
    }
    settings.c_iflag &= ~HUPCL;
    if(nr)
    settings.c_iflag |= HUPCL;
    result = tcsetattr(x->comhandle, TCSANOW, &settings);
    if (result < 0)
    {
        pd_error(x,"[komport] could not set HUPCL");
        return 0;
    }
    x->hupcl = nr;
    return 1;
}

static int open_serial(unsigned int com_num, t_komport *x)
{
    int             fd;
    unsigned int    i;
    struct termios  *old = &(x->oldcom_termio);
    struct termios  *new = &(x->com_termio);
    float           *baud = &(x->baud);
    glob_t          glob_buffer;

    /* if com_num == USE_DEVICENAME, use device name directly, else try port # */
    if((com_num != USE_DEVICENAME)&&(com_num >= komport_MAX))
    {
        post("[komport] ** WARNING ** port %d not valid, must be between 0 and %d",
            com_num, komport_MAX - 1);
        return INVALID_HANDLE_VALUE;
    }
    /*  post("[komport] globbing %s",x->serial_device_prefix);*/
    /* get the device path based on the port# and the glob pattern */
    switch( glob( x->serial_device_prefix, 0, NULL, &glob_buffer ) )
    {
        case GLOB_NOSPACE:
            error("[komport] out of memory for \"%s\"",x->serial_device_prefix);
            break;
#ifdef GLOB_ABORTED
        case GLOB_ABORTED:
            error("[komport] aborted \"%s\"",x->serial_device_prefix);
            break;
#endif
#ifdef GLOB_NOMATCH
        case GLOB_NOMATCH:
            error("[komport] no serial devices found for \"%s\"",x->serial_device_prefix);
            break;
#endif
    }
    if (com_num == USE_DEVICENAME)
    { /* if possible, find the index of the devicename */
        for (i = 0; i < glob_buffer.gl_pathc; ++i)
        {
            if (0 == strcmp(x->serial_device->s_name, glob_buffer.gl_pathv[i]))
            {
                com_num = i;
                break;
            }
        }
    }
    else if(com_num < glob_buffer.gl_pathc)
        x->serial_device = gensym(glob_buffer.gl_pathv[com_num]);
    else
    {
        post("[komport] ** WARNING ** port #%d does not exist! (max == %d)",
            com_num,glob_buffer.gl_pathc - 1);
        return INVALID_HANDLE_VALUE;
    }
    globfree( &(glob_buffer) );

    if((fd = open(x->serial_device->s_name, OPENPARAMS)) == INVALID_HANDLE_VALUE)
    {
        error("[komport] ** ERROR ** could not open device %s:\n failure(%d): %s\n",
            x->serial_device->s_name,errno,strerror(errno));
        return INVALID_HANDLE_VALUE;
    }

    /* set no wait on any operation */
    fcntl(fd, F_SETFL, FNDELAY);

    /*   Save the Current Port Configuration  */
    if(tcgetattr(fd, old) == -1 || tcgetattr(fd, new) == -1)
    {
        error("[komport] ** ERROR ** could not get termios-structure of device %s\n",
            x->serial_device->s_name);
        close(fd);
        return INVALID_HANDLE_VALUE;
    }

    /* Setup the new port configuration...NON-CANONICAL INPUT MODE
    .. as defined in termios.h */

    /* enable input and ignore modem controls */
    new->c_cflag |= (CREAD | CLOCAL);

    /* always nocanonical, this means raw i/o no terminal */
    new->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* no post processing */
    new->c_oflag &= ~OPOST;

    /* setup to return after 0 seconds
        ..if no characters are received
        TIME units are assumed to be 0.1 secs */
    /* not needed anymore ??? in new termios in linux
    new->c_cc[VMIN] = 0;
    new->c_cc[VTIME] = 0;
    */

    /* defaults, see input */

    set_bits(x, x->data_bits);      /* CS8 */
    set_stopflag(x, x->stop_bits);  /* ~CSTOPB */
    set_ctsrts(x, x->ctsrts);  /* ~CRTSCTS;*/
    set_xonxoff(x, x->xonxoff); /* (IXON | IXOFF | IXANY) */
    set_baudrate(x, *baud);

    if(tcsetattr(fd, TCSAFLUSH, new) != -1)
    {
        post("[komport] opened serial line device %d (%s)\n",
            com_num,x->serial_device->s_name);
    }
    else
    {
        error("[komport] ** ERROR ** could not set params to ioctl of device %s\n",
            x->serial_device->s_name);
        close(fd);
        return INVALID_HANDLE_VALUE;
    }
    x->komport = com_num; /* output at next komport_tick */
    return fd;
}

static int close_serial(t_komport *x)
{
    struct termios *tios = &(x->com_termio);
    HANDLE         fd = x->comhandle;

    if(fd != INVALID_HANDLE_VALUE)
    {
        tcsetattr(fd, TCSANOW, tios);
        close(fd);
        post("[komport] closed %s",x->serial_device->s_name);
    }
    return INVALID_HANDLE_VALUE;
}

static int set_serial(t_komport *x)
{
    if(tcsetattr(x->comhandle, TCSAFLUSH, &(x->com_termio)) == -1)
        return 0;
    return 1;
}

static int komport_get_dsr(t_komport *x)
{
    short  dsr_state = 0;

    if (x->comhandle != INVALID_HANDLE_VALUE)
    {
        int status;/*dsr outlet*/
        /* get the DSR input state and if it's changed, output it */
        ioctl(x->comhandle, TIOCMGET, &status);/*dsr outlet*/
        dsr_state = ((status&TIOCM_LE)!=0);/* read the DSR input line */
    }
    return dsr_state;
}

int komport_get_cts(t_komport *x)
{
    short  cts_state = 0;

    if (x->comhandle != INVALID_HANDLE_VALUE)
    {
        int status;/*cts outlet*/
        /* get the CTS input state and if it's changed, output it */
        ioctl(x->comhandle, TIOCMGET, &status);
        cts_state = ((status&TIOCM_CTS)!=0);/* read the CTS input line */
    }
    return cts_state;
}

/* ------------------- serial pd methods --------------------------- */
static void komport_pollintervall(t_komport *x, t_floatarg g)
{
    if (g < 1) g = 1;
    x->x_deltime = g;
}

static void komport_tick(t_komport *x)
{

    int  fd = x->comhandle;
    int          err;

    x->x_hit = 0;

    if(fd != INVALID_HANDLE_VALUE)
    { /* while there are bytes, read them and send them out, ignore errors (!??) */

        fd_set          com_rfds;
        int             count = 0;
        int             i;
        int             whicherr = 0;

        FD_ZERO(&com_rfds);
        FD_SET(fd,&com_rfds);
        while((err = select(fd+1, &com_rfds, NULL, NULL, &null_tv)) > 0)
        {
            ioctl(fd, FIONREAD, &count); /* load count with the number of bytes in the receive buffer... */
            if (count > x->x_inbuf_len) count = x->x_inbuf_len; /* ...but no more than the buffer can hold */
            /*err = read(fd,(char *) &serial_byte,1);*/
            err = read(fd,(char *)x->x_inbuf, count);/* try to read count bytes */
            if (err >= 0)
            {
                for (i = 0; i < err; ++i )
                {
                    outlet_float(x->x_data_outlet, (t_float) x->x_inbuf[i]);
                }
            }
            else whicherr = errno;
        }

        if(err < 0)
        { /* if a read error detected */
            if(x->rxerrors < 10) /* ten times max */
                post("RXERRORS on serial line (%d)\n", whicherr);
            x->rxerrors++; /* remember */
        }
/* now if anything to send, send the output buffer */
        if (0 != x->x_outbuf_wr_index)
        {

            err = write(x->comhandle,(char *)x->x_outbuf, x->x_outbuf_wr_index);
            if (err != x->x_outbuf_wr_index)
            post ("[komport]: Write returned %d, errno is %d", err, errno);

            x->x_outbuf_wr_index = 0; /* for now we just drop anything that didn't send */
        }
        if (!x->x_hit) clock_delay(x->x_clock, x->x_deltime); /* default 1 ms */
    }
}

static int write_serial(t_komport *x, unsigned char  serial_byte)
{
    if(x->comhandle == INVALID_HANDLE_VALUE)
    {
        post ("[komport]: Serial port is not open");
        return 0;         
    }
    else if(x->x_outbuf_wr_index < x->x_outbuf_len)
    {
        x->x_outbuf[x->x_outbuf_wr_index++] = serial_byte;
        return 1;
    }    
    /* handle overrun error */
    post ("[komport]: buffer is full");
    return 0;
}

static int write_serials(t_komport *x, unsigned char *serial_buf, int buf_length)
{
    int i;
    if(x->comhandle == INVALID_HANDLE_VALUE)
    {
        post ("[komport]: Serial port is not open");
        return 0;         
    }
    for (i = 0; ((i < buf_length) && (x->x_outbuf_wr_index < x->x_outbuf_len)); ++x->x_outbuf_wr_index, ++i)
        x->x_outbuf[x->x_outbuf_wr_index] = serial_buf[i];
    if (i != buf_length) post ("[komport]: buffer is full");
    return i;
}

static void komport_float(t_komport *x, t_float f)
{
    unsigned char serial_byte = ((int) f) & 0xFF; /* brutal conv */

    if (write_serial(x,serial_byte) != 1)
    {
        post("Write error, maybe TX-OVERRUNS on serial line");
    }
}

static void komport_list(t_komport *x, t_symbol *s, int argc, t_atom *argv)
{
    unsigned char   temp_array[komport_BUF_SIZE];/* arbitrary maximum list length */
    int             i, count;
    int             result;

    count = argc;
    if (argc > komport_BUF_SIZE)
    {
        post ("[komport] truncated list of %d elements to %d", argc, count);
        count = komport_BUF_SIZE;
    }
    for(i = 0; i < count; i++)
        temp_array[i] = ((unsigned char)atom_getint(argv+i))&0xFF; /* brutal conv */
    result = write_serials(x, temp_array, count);
}

static void *komport_new(t_symbol *s, int argc, t_atom *argv)
{
    t_komport test;
    t_komport *x;
    HANDLE    fd;
    const char *serial_device_prefix;
    t_float com_num = 0;
    t_float fbaud = 9600;


/* for UNIX, this is a glob pattern for matching devices  */
#ifdef __APPLE__
    serial_device_prefix = "/dev/tty.*";
#endif /* __APPLE__ */
#ifdef IRIX
    serial_device_prefix = "/dev/ttyd*";
#endif /* IRIX */
#ifdef __linux__
    /* serial: ttyS?    USB-serial: ttyUSB?   USB-CDC: ttyACM? */
    serial_device_prefix = "/dev/tty[ASU]*";
#endif /* __linux__ */

    if(argc > 0 && argv->a_type == A_FLOAT)
        com_num = atom_getfloatarg(0,argc,argv);
    if(argc > 1)
        fbaud = atom_getfloatarg(1,argc,argv);

/*	 Open the komport for RD and WR and get a handle */
/* this line should use a real serial device */
    strncpy(test.serial_device_prefix, serial_device_prefix, strlen(serial_device_prefix)+1);
    test.baud = fbaud;
    test.data_bits = 8; /* default 8 data bits */
    test.parity_bit = 0;/* default no parity bit */

    test.stop_bits = 0;/* default 1 stop bit */
    test.ctsrts = 0; /* default no hardware handshaking */
    test.xonxoff = 0; /* default no software handshaking */
    test.hupcl = 1; /* default hangup on close */
    fd = open_serial((unsigned int)com_num, &test);

    /* Now  nothing really bad could happen so we create the class */
    x = (t_komport *)pd_new(komport_class);

    x->komport = test.komport;/* com_num */
    strncpy(x->serial_device_prefix,serial_device_prefix,strlen(serial_device_prefix)+1);
    x->serial_device = test.serial_device; /* we need this so 'help' doesn't crash */

    x->baud = test.baud;
    x->data_bits = test.data_bits;
    x->parity_bit = test.parity_bit;
    x->stop_bits = test.stop_bits;
    x->ctsrts = test.ctsrts;
    x->xonxoff = test.xonxoff;
    x->hupcl = test.hupcl;
    x->comhandle = fd; /* holds the komport handle */

    if(fd == INVALID_HANDLE_VALUE )
    {
        pd_error(x, "[komport] opening serial port %g failed!", com_num);
    }
    else
    {

        /* save the old com and new com config */
        bcopy(&(test.oldcom_termio),&(x->oldcom_termio),sizeof(struct termios));
        bcopy(&(test.com_termio),&(x->com_termio),sizeof(struct termios));

    }

/* allocate memory for in and out buffers */
    x->x_inbuf = getbytes(komport_BUF_SIZE);
    if (NULL == x->x_inbuf)
    {
        pd_error(x, "[komport] unable to allocate input buffer");
        return 0;
    }
    x->x_inbuf_len = komport_BUF_SIZE;
    x->x_outbuf = getbytes(komport_BUF_SIZE);
    if (NULL == x->x_outbuf)
    {
        pd_error(x, "[komport] unable to allocate output buffer");
        return 0;
    }
    x->x_outbuf_len = komport_BUF_SIZE;
    x->x_outbuf_wr_index = 0;

    x->rxerrors = 0; /* holds the rx line errors */

    x->x_data_outlet = outlet_new(&x->x_obj, &s_float);
    x->x_status_outlet = outlet_new(&x->x_obj, &s_float);

    x->x_hit = 0;
    x->x_deltime = 1;
    x->x_clock = clock_new(x, (t_method)komport_tick);

    clock_delay(x->x_clock, x->x_deltime);

    x->verbose = 0;

    return x;
}


static void komport_free(t_komport *x)
{
    post("[komport] free serial...");
    clock_unset(x->x_clock);
    clock_free(x->x_clock);
    x->comhandle = close_serial(x);
    freebytes(x->x_inbuf, x->x_inbuf_len);
    freebytes(x->x_outbuf, x->x_outbuf_len);
}

/* ---------------- use serial settings ------------- */

static void komport_baud(t_komport *x,t_floatarg f)
{
    if(f == x->baud)
    {
        post("baudrate already %g\n",x->baud);
        return;
    }

    x->baud = set_baudrate(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(set_serial(x) == 0)
    {
        error("[komport] ** ERROR ** could not set baudrate of device %s\n",

            x->serial_device->s_name);
    }
    else if(x->verbose > 0)
        post("set baudrate of %s to %g\n",

            x->serial_device->s_name, x->baud);

}

static void komport_bits(t_komport *x,t_floatarg f)
{
    f = set_bits(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(set_serial(x) == 0)
    {
        error("[komport] ** ERROR ** could not set bits of device %s\n",

            x->serial_device->s_name);

        return;
    }
    else if(x->verbose > 0)
        post("set bits of %s to %g\n",

            x->serial_device->s_name, f);

    x->data_bits = f;
}


static void komport_parity(t_komport *x,t_floatarg f)
{
    f = set_parity(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(set_serial(x) == 0)
    {
        error("[komport] ** ERROR ** could not set extra paritybit of device %s\n",

            x->serial_device->s_name);

        return;
    }
    else if(x->verbose > 0)
        post("[komport] set extra paritybit of %s to %g\n",

            x->serial_device->s_name, f);

    x->parity_bit = f;
}

static void komport_stopbit(t_komport *x, t_floatarg f)
{
    f = set_stopflag(x, f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(set_serial(x) == 0)
    {

        error("[komport] ** ERROR ** could not set extra stopbit of device %s\n",
            x->serial_device->s_name);

        return;
    }
    else if(x->verbose > 0)

        post("[komport] set extra stopbit of %s to %g\n",
            x->serial_device->s_name, f);

    x->stop_bits = f;
}

static void komport_rtscts(t_komport *x,t_floatarg f)
{
    f = set_ctsrts(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(set_serial(x) == 0)
    {
        error("[komport] ** ERROR ** could not set rts_cts of device %s\n",
            x->serial_device->s_name);

        return;
    }
    else if(x->verbose > 0)
        post("[komport] set rts-cts of %s to %g\n",

            x->serial_device->s_name, f);

    x->ctsrts = f;
}

static void komport_dtr(t_komport *x,t_floatarg f)
{
    f = set_dtr(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(f < 0)
    {
        error("[komport] ** ERROR ** could not set dtr of device %s\n",

            x->serial_device->s_name);

    }
    else if(x->verbose > 0)
        post("[komport] set dtr of %s to %g\n",

            x->serial_device->s_name, f);

}

static void komport_rts(t_komport *x,t_floatarg f)
{
    f = set_rts(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(f < 0)
    {
        error("[komport] ** ERROR ** could not set rts of device %s\n",

            x->serial_device->s_name);

    }
    else if(x->verbose > 0)
        post("[komport] set rts of %s to %g\n",

            x->serial_device->s_name, f);

}

static void komport_xonxoff(t_komport *x,t_floatarg f)
{
    f = set_xonxoff(x,f);

    if(x->comhandle == INVALID_HANDLE_VALUE)return;

    if(set_serial(x) == 0)
    {
        error("[komport] ** ERROR ** could not set xonxoff of device %s\n",

            x->serial_device->s_name);

        return;
    }
    else if(x->verbose > 0)
        post("[komport] set xonxoff of %s to %g\n",

        x->serial_device->s_name, f);

    x->xonxoff = f;
}

static void komport_hupcl(t_komport *x,t_floatarg f)
{
#ifndef _WIN32
	set_hupcl(x,f);
#endif
}

static void komport_close(t_komport *x)
{
    clock_unset(x->x_clock);
    x->x_hit = 1;
    x->comhandle = close_serial(x);
    x->komport = -1; /* none */
    if (x->x_status_outlet != NULL) outlet_float(x->x_status_outlet, (float)x->komport);
}

static void komport_open(t_komport *x, t_floatarg f)
{
    if(x->comhandle != INVALID_HANDLE_VALUE)
        komport_close(x);

    x->comhandle = open_serial(f,x);

    clock_delay(x->x_clock, x->x_deltime);
}

/*
   dangerous but if you really have some stupid devicename ...
   you can open any file on systems if suid is set on pd be careful
*/

static void komport_devicename(t_komport *x, t_symbol *s)
{
    x->serial_device = s;
    if(x->comhandle != INVALID_HANDLE_VALUE)
        komport_close(x);

    x->comhandle = open_serial(USE_DEVICENAME,x);
    clock_delay(x->x_clock, x->x_deltime);
}

static void komport_print(t_komport *x, t_symbol *s, int argc, t_atom *argv)
{
    static char buf[256];
    char        *pch = buf;

    while(argc--)
    {
        atom_string(argv++, buf, 255);
        while(*pch != 0)
        {
            write_serial(x, *pch++);
        }
        if(argc > 0)
        {
            write_serial(x, ' ');
        }
    }
}

static void komport_enum(t_komport *x)
{
    unsigned int i;
    glob_t         glob_buffer;
    int            fd;
    struct termios test;

/* first look for registered devices in the filesystem */
    switch( glob( x->serial_device_prefix, 0, NULL, &glob_buffer ) )
    {
    case GLOB_NOSPACE:
        error("[komport] out of memory for \"%s\"",x->serial_device_prefix);
        break;
# ifdef GLOB_ABORTED
        case GLOB_ABORTED:
        error("[komport] aborted \"%s\"",x->serial_device_prefix);
        break;
# endif /* GLOB_ABORTED */
# ifdef GLOB_NOMATCH
    case GLOB_NOMATCH:
        error("[komport] no serial devices found for \"%s\"",x->serial_device_prefix);
        break;
# endif /* GLOB_NOMATCH */
    }
    for(i=0; i<glob_buffer.gl_pathc; i++)
    {
/* now try to open the device */
        if((fd = open(glob_buffer.gl_pathv[i], OPENPARAMS)) != INVALID_HANDLE_VALUE)
        {
/* now see if it has attributes */
            if ((tcgetattr(fd, &test)) != -1)
                post("\t%d\t%s", i, glob_buffer.gl_pathv[i]);// this one really exists
                close (fd);
        }
    }
}

static void komport_ports(t_komport *x)
{ /* the same as komport_enum except outputs list of available ports on status outlet */
    unsigned int    i, j = 0;
    t_atom          output_atom[2];

    glob_t          glob_buffer;
    int             fd;
    struct termios  test;

/* first look for registered devices in the filesystem */
    switch( glob( x->serial_device_prefix, 0, NULL, &glob_buffer ) )
    {
        case GLOB_NOSPACE:
            error("[komport] out of memory for \"%s\"",x->serial_device_prefix);
            break;
# ifdef GLOB_ABORTED
        case GLOB_ABORTED:
            error("[komport] aborted \"%s\"",x->serial_device_prefix);
            break;
# endif /* GLOB_ABORTED */
# ifdef GLOB_NOMATCH
        case GLOB_NOMATCH:
            error("[komport] no serial devices found for \"%s\"",x->serial_device_prefix);
            break;
# endif /* GLOB_NOMATCH */
    }
    for(i = 0; (i < glob_buffer.gl_pathc) && (j < komport_MAX); i++)
    {
/* now try to open the device */
        if((fd = open(glob_buffer.gl_pathv[i], OPENPARAMS)) != INVALID_HANDLE_VALUE)
        {
/* now see if it has attributes */
            if ((tcgetattr(fd, &test)) != -1)
            { /* output index and name as a list */
                SETFLOAT(&output_atom[0], i);
                SETSYMBOL(&output_atom[1], gensym(glob_buffer.gl_pathv[i]));
                outlet_anything( x->x_status_outlet, gensym("ports"), 2, output_atom);
            }
            close (fd);
        }
    }
}

static void komport_output_print(t_komport *x)
{
    post("[komport]: available serial ports:");
    komport_enum(x);
}


static void komport_output_status(t_komport *x, t_symbol *selector, t_float output_value)
{
    t_atom *output_atom = getbytes(sizeof(t_atom));
    SETFLOAT(output_atom, output_value);
    outlet_anything( x->x_status_outlet, selector, 1, output_atom);
    freebytes(output_atom,sizeof(t_atom));
}

static void komport_output_port_status(t_komport *x)
{
    komport_output_status(x, gensym("port"), (float)x->komport);
}

static void komport_output_dsr_status(t_komport *x)
{
    komport_output_status(x, gensym("dsr"), (float)komport_get_dsr(x));
}

static void komport_output_cts_status(t_komport *x)
{
    komport_output_status(x, gensym("cts"), (float)komport_get_cts(x));
}

static void komport_output_baud_rate(t_komport *x)
{
    komport_output_status(x, gensym("baud"), x->baud);
}

static void komport_output_parity_bit(t_komport *x)
{
    komport_output_status(x, gensym("parity"), x->parity_bit);
}

static void komport_output_stop_bits(t_komport *x)
{

    komport_output_status(x, gensym("stop"), x->stop_bits+1);

}

static void komport_output_data_bits(t_komport *x)
{
    komport_output_status(x, gensym("data"), x->data_bits);
}

static void komport_output_rtscts(t_komport *x)
{
    komport_output_status(x, gensym("rtscts"), x->ctsrts);
}

static void komport_output_xonxoff(t_komport *x)
{
    komport_output_status(x, gensym("xonxoff"), x->xonxoff);
}

static void komport_output_hupcl(t_komport *x)
{
    komport_output_status(x, gensym("hupcl"), x->hupcl);
}

static void komport_output_rxerrors(t_komport *x)
{
    komport_output_status(x, gensym("rxerrors"), x->rxerrors);
}

static void komport_output_open_status(t_komport *x)
{
    if(x->comhandle == INVALID_HANDLE_VALUE)
        komport_output_status(x, gensym("open"), 0);
    else
        komport_output_status(x, gensym("open"), 1);
}

static void komport_devices(t_komport *x)
{
    komport_output_print(x);
}

static void komport_info(t_komport *x)
{
    komport_output_open_status(x);
    komport_output_port_status(x);
    komport_output_baud_rate(x);
    komport_output_dsr_status(x);
    komport_output_cts_status(x);
    komport_output_parity_bit(x);
    komport_output_stop_bits(x);
    komport_output_data_bits(x);
    komport_output_rtscts(x);
    komport_output_xonxoff(x);
    komport_output_hupcl(x);
    komport_output_rxerrors(x);
}

/* ---------------- HELPER ------------------------- */
static void komport_verbose(t_komport *x, t_floatarg f)
{
    x->verbose = f;
    if(f > 0) post("[komport] verbose is on: %d", (int) f);
}

static void komport_help(t_komport *x)
{
    post("[komport] serial port %d (baud %g):", x->komport, x->baud);
    if(x->komport >= 0 && x->komport < komport_MAX)
    {

        post("\tdevicename: %s", x->serial_device->s_name);

    }

    post("  Methods:");
    post("   baud <baud>       ... set baudrate to nearest possible baud\n"
        "   bits <bits>       ... set number of bits (7 or 8)\n"
        "   stopbit <0|1>     ... set off|on stopbit\n"
        "   rtscts <0|1>      ... set rts/cts off|on\n"
        "   parity <0|1>      ... set parity off|on\n"
        "   xonxoff <0|1>     ... set xon/xoff off|on\n"
        "   dtr <0|1>         ... set dtr off|on\n"
        "   rts <0|1>         ... set rts off|on\n"
        "   hupcl <0|1>       ... set hang-up on close off|on\n"
        "   close             ... close device\n"
        "   open <num>        ... open device number num\n"
        "   devicename <d>    ... set device name to d (eg. /dev/ttyS8)\n"
        "   print <list>      ... print list of atoms on serial\n"
        "   pollintervall <t> ... set poll interval to t ticks\n"
        "   verbose <level>   ... for debug set verbosity to level\n"
        "   info              ... output info on status outlet\n"
        "   devices           ... post list of available devices\n"
        "   ports             ... output list of available devices on status outlet\n"
        "   help              ... post this help");
}

/* ---------------- SETUP OBJECTS ------------------ */
void komport_setup(void)
{
    komport_class = class_new(gensym("komport"), (t_newmethod)komport_new,
        (t_method)komport_free, sizeof(t_komport),
        0, A_GIMME, 0);

    class_addfloat(komport_class, (t_method)komport_float);
    class_addlist(komport_class, (t_method)komport_list);
    /*
        class_addbang(komport_class, komport_bang
    */
    class_addmethod(komport_class, (t_method)komport_baud, gensym("baud"),A_FLOAT, 0);

    class_addmethod(komport_class, (t_method)komport_bits, gensym("bits"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_stopbit, gensym("stopbit"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_rtscts, gensym("rtscts"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_dtr, gensym("dtr"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_rts, gensym("rts"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_parity, gensym("parity"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_xonxoff, gensym("xonxoff"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_hupcl, gensym("hupcl"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_close, gensym("close"), 0);
    class_addmethod(komport_class, (t_method)komport_open, gensym("open"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_devicename, gensym("devicename"), A_SYMBOL, 0);
    class_addmethod(komport_class, (t_method)komport_print, gensym("print"), A_GIMME, 0);
    class_addmethod(komport_class, (t_method)komport_pollintervall, gensym("pollintervall"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_verbose, gensym("verbose"), A_FLOAT, 0);
    class_addmethod(komport_class, (t_method)komport_help, gensym("help"), 0);
    class_addmethod(komport_class, (t_method)komport_info, gensym("info"), 0);
    class_addmethod(komport_class, (t_method)komport_devices, gensym("devices"), 0);
    class_addmethod(komport_class, (t_method)komport_ports, gensym("ports"), 0);

#ifndef _WIN32
    null_tv.tv_sec = 0; /* no wait */
    null_tv.tv_usec = 0;
#endif /* NOT _WIN32 */
    post("komport - PD external for unix/windows\n"
        "LGPL 1998-2006,  Winfried Ritsch and others (see LICENCE.txt)\n"
        "Institute for Electronic Music - Graz");
}

