/*
	SharpIR

	Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK

	From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
	
    Version : 1.0 : Guillaume Rico

	https://github.com/guillaume-rico/SharpIR

*/

#ifndef SharpIR_h
#define SharpIR_h

#define NB_SAMPLE 10

#include "Arduino.h"


//#include <sensor_msgs/Range.h>


class ZSharpIR
{
	
  public:

    ZSharpIR (int irPin, const uint32_t _sensorType);
    int distance();
	static const uint32_t GP2Y0A41SK0F = 430 ;
	static const uint32_t GP2Y0A21YK0F = 1080 ;
	static const uint32_t GP2D12_24 = 1081 ;
	static const uint32_t GP2Y0A02YK0F = 20150 ;
	static const uint32_t GP2Y0A710K0F = 100500 ;
	

     void setARefVoltage(int refV);
     void SetAnalogReadResolution(int res);
     
  private:
     int    _Adcres;
     int _refVoltage;
     
    void sort(int a[], int size);
    
    int _irPin;
    long _model;
};

#endif
