/*
	ZSharpIR

	Arduino library for retrieving distance (in mm) from the analog GP2Y0A21Y and GP2Y0A02YK,...
	Author :
	Pierre Valleau
	history :
		add ros supports
		add several model support.
		
    Original comment from Dr. Marcal Casas-Cartagena :
	inspired from :
	- https://github.com/qub1750ul/Arduino_SharpIR.git
    - https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library.git
    - https://github.com/guillaume-rico/SharpIR.git
    - https://github.com/nikv96/MDP-Arduino.git
    - https://github.com/jeroendoggen/Arduino-GP2Y0A21YK-library.git

  

*/


#include "Arduino.h"//<=should be included here #include "WMath.h"

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
/** the calibration table of 20 value,
1st is for 0cm
next is for 10cm, naxt for 20,.... up to 190cm
you can store it after calibration procedure, or restore it after power up.

*/
int  table[20];
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
int ZSharpIR::getRaw()
{
	 int ir_val[NB_SAMPLE];
	  for (int i=0; i<NB_SAMPLE; i++){
        // Read analog value
        ir_val[i] = analogRead(_irPin);
    }
    
    // Sort it 
    sort(ir_val,NB_SAMPLE);
	
	return ir_val[NB_SAMPLE / 2];
	
} 

// Read distance and compute it
int ZSharpIR::distance() {

   
   


   int ir_val=getRaw();
   return compute( ir_val) ;
}
int ZSharpIR::compute(int ir_val) { 
 int distanceMM=0;
    float current=0;
    if (_model==1080)//GP2Y0A21YK0F
		{
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 277.28 * pow(map(ir_val, 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.2045));


    }   
    else if (_model==GP2D12_24)//GP2D12_24
		{
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 24.65251 /(map(ir_val, 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0-0.1065759));

    } 
    

    
    else if (_model==20150)//GP2Y0A02YK0F
	{

        // Previous formula used by  Dr. Marcal Casas-Cartagena
        // puntualDistance=61.573*pow(voltFromRaw/1000, -1.1068);
        
        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 603.74 * pow(map(ir_val, 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.16));


    } else if (_model==430)//GP2Y0A41SK0F
	{

        // Different expressions required as the Photon has 12 bit ADCs vs 10 bit for Arduinos
          distanceMM =(int)( 120.8 * pow(map(ir_val, 0, (1<<_Adcres)-1, 0, _refVoltage)/1000.0, -1.058));

        
    } else if (_model==100500)//GP2Y0A710K0F
	{
        

          current = map(ir_val, 0, (1<<_Adcres)-1, 0, _refVoltage);

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
 else if (_model==CALIBRATED)//GP2Y0A710K0F
	{
		int index=0;
        for( index=19;index>=0 && ir_val>table[index];index--);
          distanceMM = map(ir_val, table[index], table[index+1], index*10, index*10+10);
    }
    return distanceMM;
}
/** Return the possible error 
(based on a noise/error of 5 LSB of ADC measure)
this inaccuracy isn't lineare so this function can be interesting.
so realvalue=distance() +/-getAccuracy().

*/
int ZSharpIR::getAccuracy() 
{
	
	 int ir_val=getRaw();
   return abs( compute( ir_val+5)-compute( ir_val)) ;
	 
}
/** return the max valid value of captor
*/
int ZSharpIR::getMax() 
{
	 if (_model==1080)//GP2Y0A21YK0F
	{        
       return 800;
    }   
    else if (_model==GP2D12_24)//GP2D12_24
		{        
        return 800;
    }       
    else if (_model==20150)//GP2Y0A02YK0F
	{

       return 150;


    } else if (_model==430)//GP2Y0A41SK0F
	{

       return 300;

        
    } else if (_model==100500)//GP2Y0A710K0F
	{
		return 5000;
	}
	else if (_model==CALIBRATED)//GP2Y0A710K0F
	{
		
		int previous=32767;
		int dmax=0;
		 for(int  index=19;index>=getMin()/10;index--)
		 {
		 if (table[index]<=previous)
			 dmax=index*10;
		 previous=table[index];
		 }
		return dmax;
	}
	
	return -1;
}
/** return the min valid value of captor
*/
int ZSharpIR::getMin() 
{
	 if (_model==1080)//GP2Y0A21YK0F
	{        
       return 100;
    }   
    else if (_model==GP2D12_24)//GP2D12_24
		{        
        return 100;
    }       
    else if (_model==20150)//GP2Y0A02YK0F
	{

       return 200;


    } else if (_model==430)//GP2Y0A41SK0F
	{

       return 40;

        
    } else if (_model==100500)//GP2Y0A710K0F
	{
		return 1000;
	}
	else if (_model==CALIBRATED)//GP2Y0A710K0F
	{
		
		int previous=0;
		int dmax=0;
		 for(int  index=0;index<20 ;index++)		 
		 if (table[index]>=previous)
		 {
			 dmax=index*10;
		 previous=table[index];
		 }
		 else
		return dmax;
	
	}
	
		return -1;
	
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
	#ifndef ARDUINO_ARCH_AVR
	analogReadResolution( res);
	#endif
}
int indexZsharpIR=0;
void ZSharpIR::ApplyCalibration(int atable[20])
{
	 for(int  index=0;index<20 ;index++)	
		 table[index]=atable[index];
	 _model=CALIBRATED;
}
void ZSharpIR::DisplayCalibration(Stream & Serial)
{
   Serial.println("int tableXXX[]={");
    for(int index=0;index<20;index++)
    {
    
   Serial.print("\t\t  ");  // returns it to the serial monitor
  
  Serial.print(table[index]);
if (index<19)
    Serial.println(","); 
      }
       Serial.println("\t\t};"); 
}
/** call this function to calibrate the captor
after call 20 times CalibrateNextStep() moving a captor from 0cm to 190cm by step of 10cm from the obstacle.
calibrate oonly 1 captor at a time.
*/
void ZSharpIR::CalibrateStart()
{
	indexZsharpIR=0;
}
void ZSharpIR::CalibrateNextStep()
{
	if (indexZsharpIR<20)
	{
table[indexZsharpIR]=getRaw();
indexZsharpIR++;
	}
	else
		_model=CALIBRATED;
}
 
 


#ifdef ROS_USED 
// ROS SECTION :
//char frameid[] = "/ir_ranger";
/** setup :
  At setup after NodeHandle setup, call this to initialise the topic
*/
void ZSharpIR::setup( ros::NodeHandle * myNodeHandle,	const char   *	topic )
{
  nh=myNodeHandle;
  pub_range=new ros::Publisher( topic, &range_msg);
  assert( pub_range!=0);// heap issue.
  
  bool result=nh->advertise(*pub_range);
  assert(result);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
//  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 10;
  range_msg.min_range = ((float)getMin())/1000.0 ;  // For GP2D120XJ00F only. Adjust for other IR rangers
  range_msg.max_range = ((float)getMax())/1000.0 ;   // For GP2D120XJ00F only. Adjust for other IR rangers
  
  range_msg.header.stamp = nh->now();
}
#define toNSec( t ) ((uint32_t)t.sec*1000000000ull + (uint32_t)t.nsec)
/** loop :
  on loop  before NodeHandle refresh(spinOnce), call this to update the topic
*/
void ZSharpIR::loop()
{
	ros::Time now=nh->now();

  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (toNSec( now ) ) > (50000000LL+toNSec( (range_msg.header.stamp) ))){
    range_msg.range = ((float)distance())/1000.0;
    range_msg.header.stamp = now;
    
                    signed int result=pub_range->publish(&range_msg);
           assert(result>0);
  } 
}
#endif 

