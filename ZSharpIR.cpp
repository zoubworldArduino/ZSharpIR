/*
	ZSharpIR

	Arduino library for retrieving distance (in mm) from the analog GP2Y0A21Y and GP2Y0A02YK,...

    Original comment from Dr. Marcal Casas-Cartagena :
	inspired from :
	- https://github.com/qub1750ul/Arduino_SharpIR.git
    - https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library.git
    - https://github.com/guillaume-rico/SharpIR.git
    - https://github.com/nikv96/MDP-Arduino.git
    - https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library.git

  

*/


#include "Arduino.h"
#include "WMath.h"
#include "ZSharpIR.h"

// Initialisation function
//  + irPin : is obviously the pin where the IR sensor is attached
//  + sensorModel is a int to differentiate the two sensor models this library currently supports:
//     1080 is the int for the GP2Y0A21Y and 
//     20150 is the int for GP2Y0A02YK and 
//     100500 is the long for GP2Y0A710K0F
//    The numbers reflect the distance range they are designed for (in cm)
ZSharpIR::ZSharpIR(int irPin, const uint32_t  sensorModel) {
  
    _irPin=irPin;
    _model=sensorModel;
    
    // Define pin as Input
    pinMode (_irPin, INPUT);
    _Adcres=10;
	_refVoltage=5000;
}

// Sort an array
void ZSharpIR::sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

// Read distance and compute it
int ZSharpIR::distance() {

    int ir_val[NB_SAMPLE];
    int distanceMM;
    float current;


    for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }
    
    // Sort it 
    sort(ir_val,NB_SAMPLE);

    
    if (_model==1080)//GP2Y0A21YK0F
		{
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 277.28 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045));


    }   
    else if (_model==GP2D12_24)//GP2D12_24
		{
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 24.65251 /(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0-0.1065759));

    } 
    

    
    else if (_model==20150)//GP2Y0A02YK0F
	{

        // Previous formula used by  Dr. Marcal Casas-Cartagena
        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 603.74 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.16));


    } else if (_model==430)//GP2Y0A41SK0F
	{

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 120.8 * pow(map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.058));

        
    } else if (_model==100500)//GP2Y0A710K0F
	{
        

          current = map(ir_val[NB_SAMPLE / 2], 0, (1<<_Adcres)-1, 0, _refVoltage);

        // use the inverse number of distance like in the datasheet (1/L)
        // y = mx + b = 137500*x + 1125 
        // x = (y - 1125) / 137500
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
        if (current < 1400 || current > 3300) {
          //false data
          distanceMM = 0;
        } else {
          distanceMM =(int)( 10.0 / (((current - 1125.0) / 1000.0) / 137.5));
        }
    }

    return distanceMM;
}




/// <summary>
/// setARefVoltage:set the ADC reference voltage: (default value: 5000mV, set to 3300mV, typically 3.3 on Arduino boards)
/// </summary>
void ZSharpIR::setARefVoltage(int refV)
{

	_refVoltage=refV;
}
/// <summary>
/// SetAnalogReadResolution:set the ADC resolution : (default value: 10, set to 12, typically 10 on Arduino boards)
/// </summary>
void ZSharpIR::SetAnalogReadResolution(int res)
{

	_Adcres=res;
	analogReadResolution( res);
}