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
    
private:

    xthConfig			config;
    xthParameters		parameters;
	xthDataspace		dataspace;
    
    xthState 			state;			// store current state of xth
    
};