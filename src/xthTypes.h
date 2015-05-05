//////////////////////////////////////
//  xthTypes.h
//
//
//


#include <iostream>
#define LOGTEN 2.302585092994

using namespace std;

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