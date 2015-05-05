///////////////////////////////////////////////////////////////////////
//
//  xth functions
//
///////////////////////////////////////////////////////////////////////

#include "xth.h"

using namespace std;

static float curveDrawCurve[100] = {-0.979999, -0.979999, -0.979999, -0.979999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.999999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.979999, -0.959999, -0.959999, -0.939999, -0.939999, -0.919999, -0.919999, -0.899999, -0.879999, -0.859999, -0.839999, -0.819999, -0.739999, -0.34, -0.24, -0.02, 0.0799999, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24, 0.26, 0.28, 0.28, 0.3, 0.32, 0.34, 0.36, 0.38, 0.4, 0.4, 0.42, 0.44, 0.46, 0.48, 0.5, 0.52, 0.559999, 0.579999, 0.599999, 0.619999, 0.639999, 0.659999, 0.679999, 0.699999, 0.719999, 0.739999, 0.779999, 0.779999, 0.799999, 0.839999, 0.839999, 0.859999, 0.879999, 0.899999, 0.919999, 0.939999, 0.939999, 0.959999, 0.979999, 1.02};


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
	
}



// Destructor
xth::~xth(){

    clear();
    
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

