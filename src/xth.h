///////////////////////////////////////////////////////////////////////
//
//  xth class
//
///////////////////////////////////////////////////////////////////////


#include "xthTypes.h"
#include <iostream>
#include <math.h>
#include <float.h>


using namespace std;


class xth{
	
public:
	
    enum xthState{
        STATE_PAUSED = 0, STATE_RUNNING = 1, STATE_ERROR = -1
    };
    
	// constructor of the xth instance
	xth();
    xth(xthConfig _config);
    
	// destructor
    ~xth();
	
    void setup();
    void setup(xthConfig _config);

    //////////////////////////
    // SIGNAL				 //
	//////////////////////////
    
	float anlzRms(float *block, int n);
	float anlzSnap(float *block, int n);
	
    
    //////////////////////////
    // FILTERING			 //
	//////////////////////////
    
	float flowAscendent(float input);
	float flowAverage(float input);
	float flowBetween(float input);
	float curveDraw(float input);
	float smoothControlParams(float input);

    //////////////////////////
    // INFO					 //
	//////////////////////////
	
	float flowLastMin(float input);
	float flowLastMax(float input);

    //////////////////////////
    // SCALING				 //
	//////////////////////////
	
	float scaleLin(float input, float top, float bottom, float topInput);
	
	
	// reset xth
	void clear();
 

    // STATE

    xthState getState();
    void setState(xthState _state);

    // CONFIG

    void setConfig(xthConfig _config);
    xthConfig getConfig();
    
    // PARAMETERS
    
    void setParameters(xthParameters _parameters);
    xthParameters getParameters();

    // DATASPACE
    
    void setDataspace(xthDataspace _dataspace);
    xthDataspace getDataspace();


    // RECEIVE
    
    void setMmgData(float raw, float nat, float soft, float lin, float tanh, float max);
    xthMmg getMmgData();
  
  	void setTempData(float val);
    float getTempData();
    
    void setAccelData(float accX, float accY, float accZ);
    xthAccel getAccelData();

    // TRANSMIT
    
    void sendOscMmgData(xthMmg mmgVals);
    void sendOscTempData(float tempVal);
    void sendOscAccelData(xthAccel accelVals);

    void sendMidiMmgData(xthMmg mmgVals);
    void sendMidiTempData(float temp);
    void sendMidiAccelData(xthAccel accelVals);


    //////////////////////////
    // SERIAL				 //
	//////////////////////////
	static int open_serial(unsigned int com_num, xthSerial *serial);
	static int close_serial(xthSerial *serial);

	static float set_baudrate(xthSerial *serial, float baud);
	static float set_bits(xthSerial *serial, int nr);
	static float set_parity(xthSerial *serial,int n);
	static float set_stopflag(xthSerial *serial, float nr);
	static int set_ctsrts(xthSerial *serial, int nr);
	static int set_dtr(xthSerial *serial, int nr);
	static int set_rts(xthSerial *serial, int nr);
	static int set_xonxoff(xthSerial *serial, int nr);
	static int set_hupcl(xthSerial *serial, int nr);



private:

    xthConfig			config;
    xthParameters		parameters;
	xthDataspace		dataspace;
    xthSerial			testSerial;
    xthSerial			serial;
    
    xthState 			state;			// store current state of xth
    
};