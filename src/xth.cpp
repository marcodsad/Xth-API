///////////////////////////////////////////////////////////////////////
//
//  xth functions
//
///////////////////////////////////////////////////////////////////////

#include "xth.h"

using namespace std;

static float curveDrawCurve[100] = {-0.979999, -0.979999, -0.979999, -0.979999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.959999, -0.959999, -0.939999, -0.939999, -0.919999, -0.919999, -0.899999, -0.879999, -0.859999, -0.839999, -0.819999, -0.739999, -0.34, -0.24, -0.02, 0.0799999, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24, 0.26, 0.28, 0.28, 0.3, 0.32, 0.34, 0.36, 0.38, 0.4, 0.4, 0.42, 0.44, 0.46, 0.48, 0.5, 0.52, 0.559999, 0.579999, 0.599999, 0.619999, 0.639999, 0.659999, 0.679999, 0.699999, 0.719999, 0.739999, 0.779999, 0.779999, 0.799999, 0.839999, 0.839999, 0.859999, 0.879999, 0.899999, 0.919999, 0.939999, 0.939999, 0.959999, 0.979999, 1.02};





// serial defines
#define HANDLE int
#define INVALID_HANDLE_VALUE -1

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



#define OPENPARAMS (O_RDWR|O_NDELAY|O_NOCTTY)
#define BAUDRATE_230400 B230400
#define BAUDRATE_115200 B115200
#define BAUDRATE_57600  B57600
#define BAUDRATE_38400  B38400

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

}; // holds the baud rate selections


static
long baudspeedbittable[] =
{
	BAUDRATE_230400,
	BAUDRATE_115200,  // CPU SPECIFIC
	BAUDRATE_57600,   // CPU SPECIFIC
	BAUDRATE_38400,   // CPU SPECIFIC
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



//--------------------------------------------------------------
xth::xth(){
}

//--------------------------------------------------------------
xth::xth(xthConfig _config){
    setup(_config);
}

//--------------------------------------------------------------
void xth::setup(){
    
    // use defualt parameters
    
    xthConfig defaultConfig;
    xthDataspace defaultDataspace;
    xthSerial defaultSerial;
    
    setup(defaultConfig);
}

// THESE TWO SETUP FUNCTIONS ARE FOR 2 CASES, ONE (ABOVE) WHERE NO CONFIG IS GIVEN, AND ONE WHERE IT IS. CURRENTLY ALWAYS GIVING CONFIG IN CALLS
//--------------------------------------------------------------
void xth::setup(xthConfig _config){
 
 	int i;
 	
    clear();
    
    config = _config;
    
    state = STATE_PAUSED;

	config.samplerate = 44100;
	config.ledActive = false;
	config.mmgActive = true;
	config.tempActive = false;
	config.accelActive = false;
	config.gyroActive = false;
    config.magnetActive = false;
    
    // default parameters
    parameters.testElement = 0;

    dataspace.flowAscendentStorage = 0;

    dataspace.flowAverageStorage[0] = 0;
    dataspace.flowAverageStorage[1] = 0;

    dataspace.flowBetweenBounds[0] = 0;
    dataspace.flowBetweenBounds[1] = 100;

    dataspace.flowLastMin = FLT_MAX;
    dataspace.flowLastMax = FLT_MIN;
      
	for(i=0; i<100; i++)
	    dataspace.curveDrawCurve[i] = curveDrawCurve[i];
	    
	dataspace.smoothFactor = 0.03;
	dataspace.smoothPrevIn = 0;	
	dataspace.smoothPrevOut = 0;
	
    dataspace.mmgVals.raw = 0;
    dataspace.mmgVals.nat = 0;
    dataspace.mmgVals.soft = 0;
    dataspace.mmgVals.lin = 0;
    dataspace.mmgVals.tanh = 0;
    dataspace.mmgVals.max = 0;
    
	dataspace.tempVal = 0;
	dataspace.accelVals.x = 0;
	dataspace.accelVals.y = 1;
	dataspace.accelVals.z = 2;
	
	
	// default serial params
    HANDLE    fd;
    const char *serial_device_prefix;
    float com_num = 2;
    float fbaud = 115200;


/* for UNIX, this is a glob pattern for matching devices  */
#ifdef __APPLE__
    serial_device_prefix = "/dev/tty.*";
#endif /* __APPLE__ */
#ifdef __linux__
    /* serial: ttyS?    USB-serial: ttyUSB?   USB-CDC: ttyACM? */
    serial_device_prefix = "/dev/tty[ASU]*";
#endif /* __linux__ */




/*	 Open the komport for RD and WR and get a handle */
/* this line should use a real serial device */
    strncpy(testSerial.serial_device_prefix, serial_device_prefix, strlen(serial_device_prefix)+1);
    testSerial.baud = fbaud;
    testSerial.data_bits = 8; /* default 8 data bits */
    testSerial.parity_bit = 0;/* default no parity bit */

    testSerial.stop_bits = 0;/* default 1 stop bit */
    testSerial.ctsrts = 0; /* default no hardware handshaking */
    testSerial.xonxoff = 0; /* default no software handshaking */
    testSerial.hupcl = 1; /* default hangup on close */
    fd = open_serial((unsigned int)com_num, &testSerial);

    /* Now  nothing really bad could happen so we create the serial instance */
    xthSerial *serial;

    serial->komport = testSerial.komport;/* com_num */
    strncpy(serial->serial_device_prefix,serial_device_prefix,strlen(serial_device_prefix)+1);
    serial->serial_device = testSerial.serial_device;

    serial->baud = testSerial.baud;
    serial->data_bits = testSerial.data_bits;
    serial->parity_bit = testSerial.parity_bit;
    serial->stop_bits = testSerial.stop_bits;
    serial->ctsrts = testSerial.ctsrts;
    serial->xonxoff = testSerial.xonxoff;
    serial->hupcl = testSerial.hupcl;
    serial->comhandle = fd; /* holds the komport handle */

    if(fd == INVALID_HANDLE_VALUE )
    {
        fprintf (stderr, "opening serial port %g failed!\n", com_num);
    }
    else
    {

        /* save the old com and new com config */
        bcopy(&(testSerial.oldcom_termio),&(serial->oldcom_termio),sizeof(struct termios));
        bcopy(&(testSerial.com_termio),&(serial->com_termio),sizeof(struct termios));

    }



/* allocate memory for in and out buffers */
    serial->x_inbuf =(unsigned char *) malloc(komport_BUF_SIZE*sizeof(unsigned char));
    if (NULL == serial->x_inbuf)
    {
        fprintf (stderr, "unable to allocate input buffer\n");
    }
    serial->x_inbuf_len = komport_BUF_SIZE;
    serial->x_outbuf =(unsigned char *) malloc(komport_BUF_SIZE*sizeof(unsigned char));
    if (NULL == serial->x_outbuf)
    {
        fprintf (stderr, "unable to allocate output buffer\n");
    }
    serial->x_outbuf_len = komport_BUF_SIZE;
    serial->x_outbuf_wr_index = 0;

    serial->rxerrors = 0; /* holds the rx line errors */

    serial->x_hit = 0;
    serial->x_deltime = 1;
    serial->verbose = 0;
	
}



// Destructor
xth::~xth(){

    clear();
    
}



////////////////////////////////////////////////////////////////
//
// SYMBOL FUNCTIONS
//
////////////////////////////////////////////////////////////////
#define HASHSIZE 1024

static t_xthSymbol *symhash[HASHSIZE];

t_xthSymbol *xthDogensym(const char *s, t_xthSymbol *oldsym)
{
    t_xthSymbol **sym1, *sym2;
    unsigned int hash = 5381;
    int length = 0;
    const char *s2 = s;
    while (*s2) /* djb2 hash algo */
    {
        hash = ((hash << 5) + hash) + *s2;
        length++;
        s2++;
    }
    sym1 = symhash + (hash & (HASHSIZE-1));
    while (sym2 = *sym1)
    {
        if (!strcmp(sym2->s_name, s)) return(sym2);
        sym1 = &sym2->s_next;
    }
    if (oldsym) sym2 = oldsym;
    else
    {
        sym2 = (t_xthSymbol *)calloc(1, sizeof(*sym2));
        sym2->s_name = (char *)calloc(length+1, sizeof(*sym2));
        sym2->s_next = 0;
        sym2->s_thing = 0;
        strcpy(sym2->s_name, s);
    }
    *sym1 = sym2;
    return (sym2);
}

t_xthSymbol *xthGensym(const char *s)
{
    return(xthDogensym(s, 0));
}





////////////////////////////////////////////////////////////////
//
// SERIAL FUNCTIONS
//
////////////////////////////////////////////////////////////////

static long get_baud_ratebits(float *baud)
{
    int i = 0;

    while(i < BAUDRATETABLE_LEN && baudratetable[i] > *baud) i++;

    if(baudratetable[i] != *baud)
        fprintf(stderr, "%d not valid, using closest value: %d\n", *baud, baudratetable[i]);

    /* nearest Baudrate finding */
    if(i==BAUDRATETABLE_LEN ||  baudspeedbittable[i] < 0)
    {
        fprintf(stderr, "*Warning* The baud rate %d is not supported or out of range, using 9600\n",*baud);
        i = 8;
    }
    *baud =  baudratetable[i];
    fprintf(stderr, "get_baud_ratebits: %g\n", *baud);

    return baudspeedbittable[i];
}

static float set_baudrate(xthSerial *serial, float baud)
{
    struct termios  *tio = &(serial->com_termio);
    speed_t         baudbits = get_baud_ratebits(&baud);
    fprintf(stderr, "set_baudrate baudbits: %d\n", baudbits);
    if( cfsetispeed(tio, baudbits) != 0 )
        fprintf(stderr, "ERROR failed to set bitrate: %d\n", baudbits);
    if( cfsetospeed(tio, baudbits) != 0 )
        fprintf(stderr, "ERROR failed to set bitrate: %d\n", baudbits);

    return baud;
}

/* bits are 5,6,7,8(default) */

static float set_bits(xthSerial *serial, int nr)
{
    struct termios *tio = &(serial->com_termio);
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
static float set_parity(xthSerial *serial,int n)
{
    struct termios *tio = &(serial->com_termio);

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
static float set_stopflag(xthSerial *serial, float f_nr)
{
    int nr = (int)f_nr;
    struct termios *tio = &(serial->com_termio);

    if(nr == 1)
    {
        tio->c_cflag |= CSTOPB;
        return 1;
    }
    else tio->c_cflag &= ~CSTOPB;

    return 0;
}

/* never tested */
static int set_ctsrts(xthSerial *serial, int nr)
{
    struct termios *tio = &(serial->com_termio);

    if(nr == 1)
    {
        tio->c_cflag |= CRTSCTS;
        return 1;
    }
    tio->c_cflag &= ~CRTSCTS;
    return 0;
}

static int set_dtr(xthSerial *serial, int nr)
{
    int fd = serial->comhandle;
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

static int set_rts(xthSerial *serial, int nr)
{
    int fd = serial->comhandle;
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

static int set_xonxoff(xthSerial *serial, int nr)
{
    struct termios *tio = &(serial->com_termio);

    if(nr == 1)
    {
        tio->c_iflag |= (IXON | IXOFF | IXANY);
        return 1;
    }

    tio->c_iflag &= ~IXON & ~IXOFF &  ~IXANY;
    return 0;
}

static int set_hupcl(xthSerial *serial, int nr)
{
    struct termios  settings;
    int             result;

    result = tcgetattr(serial->comhandle, &settings);
    if (result < 0)
    {
        fprintf(stderr, "error in tcgetattr\n");
        return 0;
    }
    settings.c_iflag &= ~HUPCL;
    if(nr)
    settings.c_iflag |= HUPCL;
    result = tcsetattr(serial->comhandle, TCSANOW, &settings);
    if (result < 0)
    {
        fprintf(stderr,"could not set HUPCL\n");
        return 0;
    }
    serial->hupcl = nr;
    return 1;
}

static int open_serial(unsigned int com_num, xthSerial *serial)
{
    int             fd;
    unsigned int    i;
    struct termios  *old = &(serial->oldcom_termio);
    struct termios  *newone = &(serial->com_termio);
    float           *baud = &(serial->baud);
    glob_t          glob_buffer;

    /* if com_num == USE_DEVICENAME, use device name directly, else try port # */
    if((com_num != USE_DEVICENAME)&&(com_num >= komport_MAX))
    {
        fprintf(stderr, "** WARNING ** port %d not valid, must be between 0 and %d\n",
            com_num, komport_MAX - 1);
        return INVALID_HANDLE_VALUE;
    }
    /*  post("[komport] globbing %s",x->serial_device_prefix);*/
    /* get the device path based on the port# and the glob pattern */
    switch( glob( serial->serial_device_prefix, 0, NULL, &glob_buffer ) )
    {
        case GLOB_NOSPACE:
            fprintf(stderr, "out of memory for \"%s\"\n",serial->serial_device_prefix);
            break;
#ifdef GLOB_ABORTED
        case GLOB_ABORTED:
            fprintf(stderr, "aborted \"%s\"\n",serial->serial_device_prefix);
            break;
#endif
#ifdef GLOB_NOMATCH
        case GLOB_NOMATCH:
            fprintf(stderr, "no serial devices found for \"%s\"\n",serial->serial_device_prefix);
            break;
#endif
    }
    if (com_num == USE_DEVICENAME)
    { /* if possible, find the index of the devicename */
        for (i = 0; i < glob_buffer.gl_pathc; ++i)
        {
            if (0 == strcmp(serial->serial_device->s_name, glob_buffer.gl_pathv[i]))
            {
                com_num = i;
                break;
            }
        }
    }
    else if(com_num < glob_buffer.gl_pathc)
        serial->serial_device = xthGensym(glob_buffer.gl_pathv[com_num]);
    else
    {
        fprintf(stderr, "** WARNING ** port #%d does not exist! (max == %d)\n",
            com_num,glob_buffer.gl_pathc - 1);
        return INVALID_HANDLE_VALUE;
    }
    globfree( &(glob_buffer) );

    if((fd = open(serial->serial_device->s_name, OPENPARAMS)) == INVALID_HANDLE_VALUE)
    {
        fprintf(stderr, "** ERROR ** could not open device %s:\n failure(%d): %s\n",
            serial->serial_device->s_name,errno,strerror(errno));
        return INVALID_HANDLE_VALUE;
    }

    /* set no wait on any operation */
    fcntl(fd, F_SETFL, FNDELAY);

    /*   Save the Current Port Configuration  */
    if(tcgetattr(fd, old) == -1 || tcgetattr(fd, newone) == -1)
    {
        fprintf(stderr, "** ERROR ** could not get termios-structure of device %s\n",
            serial->serial_device->s_name);
        close(fd);
        return INVALID_HANDLE_VALUE;
    }

    /* Setup the new port configuration...NON-CANONICAL INPUT MODE
    .. as defined in termios.h */

    /* enable input and ignore modem controls */
    newone->c_cflag |= (CREAD | CLOCAL);

    /* always nocanonical, this means raw i/o no terminal */
    newone->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    /* no post processing */
    newone->c_oflag &= ~OPOST;

    /* setup to return after 0 seconds
        ..if no characters are received
        TIME units are assumed to be 0.1 secs */
    /* not needed anymore ??? in new termios in linux
    newone->c_cc[VMIN] = 0;
    newone->c_cc[VTIME] = 0;
    */

    /* defaults, see input */

    set_bits(serial, serial->data_bits);      /* CS8 */
    set_stopflag(serial, serial->stop_bits);  /* ~CSTOPB */
    set_ctsrts(serial, serial->ctsrts);  /* ~CRTSCTS;*/
    set_xonxoff(serial, serial->xonxoff); /* (IXON | IXOFF | IXANY) */
    set_baudrate(serial, *baud);

    if(tcsetattr(fd, TCSAFLUSH, newone) != -1)
    {
        fprintf(stderr, "opened serial line device %d (%s)\n",
            com_num,serial->serial_device->s_name);
    }
    else
    {
        fprintf(stderr, "** ERROR ** could not set params to ioctl of device %s\n",
            serial->serial_device->s_name);
        close(fd);
        return INVALID_HANDLE_VALUE;
    }
    serial->komport = com_num; /* output at next komport_tick */
    return fd;
}

static int close_serial(xthSerial *serial)
{
    struct termios *tios = &(serial->com_termio);
    HANDLE         fd = serial->comhandle;

    if(fd != INVALID_HANDLE_VALUE)
    {
        tcsetattr(fd, TCSANOW, tios);
        close(fd);
        fprintf(stderr, "closed %s",serial->serial_device->s_name);
    }
    return INVALID_HANDLE_VALUE;
}













////////////////////////////////////////////////////////////////
//
// SIGNAL FUNCTIONS
//
////////////////////////////////////////////////////////////////

//--------------------------------------------------------------
float xth::anlzRms(float *block, int n){

	float hannVal, sum, output;
	int i;
	
	sum = 0;
	
	for(i=0; i<n; i++)
		sum += block[i] * block[i];
	
	output = sum/n;
	
    output = 100 + 10./LOGTEN * log(output);
	output = output < 0 ? 0 : output;
	
	output = scaleLin(output, 1, 0, 100);
	
	return(output);
}

//--------------------------------------------------------------
float xth::anlzSnap(float *block, int n){
	int idx;
	
	// return a sample at the midpoint of the window
	idx = n/2;
	
	return(block[idx]);
}


////////////////////////////////////////////////////////////////
//
// DATA FLOW FUNCTIONS
//
////////////////////////////////////////////////////////////////

//--------------------------------------------------------------
float xth::flowAscendent(float input){
	float diff;
	diff = input - dataspace.flowAscendentStorage;
	dataspace.flowAscendentStorage = input;
	
	if(diff>0)
		return(input);
	else
		return(0);
}

//--------------------------------------------------------------
float xth::flowAverage(float input){

	float sum;
	
	sum = dataspace.flowAverageStorage[0] + dataspace.flowAverageStorage[1];
	dataspace.flowAverageStorage[1] = dataspace.flowAverageStorage[0];
	dataspace.flowAverageStorage[0] = input;
	
	sum /= 2;
	
	return(sum);

}

//--------------------------------------------------------------
float xth::flowBetween(float input){

	float lower, upper;
	
	lower = dataspace.flowBetweenBounds[0];
	upper = dataspace.flowBetweenBounds[1];

	if(input>=lower && input<upper)
		return(input);
	else
		return(FLT_MAX);
}

//--------------------------------------------------------------
float xth::flowLastMin(float input){

	if(input < dataspace.flowLastMin)
		dataspace.flowLastMin = input;

	return(dataspace.flowLastMin);
}

//--------------------------------------------------------------
float xth::flowLastMax(float input){

	if(input > dataspace.flowLastMax)
		dataspace.flowLastMax = input;

	return(dataspace.flowLastMax);
}



////////////////////////////////////////////////////////////////
//
// SCALING/SMOOTHING FUNCTIONS
//
////////////////////////////////////////////////////////////////

//--------------------------------------------------------------
float xth::curveDraw(float input){

	float output;
	int idx;
	
	input *= 100;
		
	if(input<0)
	{
		input *= -1;
		
		input = input<0 ? 0 : input;
		input = input>99 ? 99 : input;
	
		idx = input+0.5;
		
		output = dataspace.curveDrawCurve[idx];
		output += 1;
		output /= 2;
		output *= -1;
	}
	else
	{
		input = input>99 ? 99 : input;

		idx = input+0.5;

		output = dataspace.curveDrawCurve[idx];
		output += 1;
		output /= 2;
	};
	
	return(output);
}

//--------------------------------------------------------------
float xth::smoothControlParams(float input){

	float output, prevIn, prevOut, smoothFactor;
	
	prevIn = dataspace.smoothPrevIn;
	prevOut = dataspace.smoothPrevOut;
	smoothFactor = dataspace.smoothFactor;
	
	output = smoothFactor*prevIn + (1-smoothFactor)*prevOut;

	dataspace.smoothPrevIn = input;	
	dataspace.smoothPrevOut = output;
	
	output = fabs(output);
	output = scaleLin(output, 1, 0, 0.8);
	
	return(output);

}

//--------------------------------------------------------------
float xth::scaleLin(float input, float top, float bottom, float topInput){

	float scaleVal, output;
	
	scaleVal = (top-bottom)/topInput;
	
	output = input*scaleVal + bottom;

	return(output);
}


////////////////////////////////////////////////////////////////
//
// GET & SET FUNCTIONS FOR ALL INTERNAL VALUES
//
////////////////////////////////////////////////////////////////

// STATE

//--------------------------------------------------------------
void xth::setState(xthState _state){
    state = _state;
}

//--------------------------------------------------------------
xth::xthState xth::getState(){
    return state;
}


// CONFIGURATION

//--------------------------------------------------------------
void xth::setConfig(xthConfig _config){
    config = _config;
}

//--------------------------------------------------------------
xthConfig xth::getConfig(){
    return config;
}


// Clear the internal data
void xth::clear(){
    
    state = STATE_PAUSED;
      
}


// PARAMETERS

//--------------------------------------------------------------
void xth::setParameters(xthParameters _parameters){
    parameters = _parameters;
}

xthParameters xth::getParameters(){
    return parameters;
}


// DATASPACE

//--------------------------------------------------------------
void xth::setDataspace(xthDataspace _dataspace){
    dataspace = _dataspace;
}

xthDataspace xth::getDataspace(){
    return dataspace;
}


////////////////////////////////////////////////////////////////
//
// RX/TX
//
////////////////////////////////////////////////////////////////

// RECEIVE

// need funcs for fetching data streams from device, and getters/setters specific to data streams

void xth::setMmgData(float raw, float nat, float soft, float lin, float tanh, float max){
    dataspace.mmgVals.raw = raw;
    dataspace.mmgVals.nat = nat;
    dataspace.mmgVals.soft = soft;
    dataspace.mmgVals.lin = lin;
    dataspace.mmgVals.tanh = tanh;
    dataspace.mmgVals.max = max;
}


xthMmg xth::getMmgData(){
    return dataspace.mmgVals;
}

void xth::setTempData(float val){
    dataspace.tempVal = val;
}

float xth::getTempData(){
    return dataspace.tempVal;
}

void xth::setAccelData(float accX, float accY, float accZ){
    dataspace.accelVals.x = accX;
    dataspace.accelVals.y = accY;
    dataspace.accelVals.z = accZ;
}


xthAccel xth::getAccelData(){
    return dataspace.accelVals;
}


// TRANSMIT
void xth::sendOscMmgData(xthMmg mmgVals){
	// send dataspace.mmgVal
}

void xth::sendOscTempData(float tempVal){
	// send dataspace.tempVal
}

void xth::sendOscAccelData(xthAccel accelVals){
	// send dataspace.accelVals
}

void xth::sendMidiMmgData(xthMmg mmgVals){
	// send dataspace.mmgVal
}

void xth::sendMidiTempData(float tempVal){
	// send dataspace.tempVal
}

void xth::sendMidiAccelData(xthAccel accelVals){
	// send dataspace.accelVals
}   

