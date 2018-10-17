/**
Example of code than use ROS as system

*/
#define  ROS_USED 
#include <ZSharpIR.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

#include <Wire.h>
#include <SPI.h>
#include <variant.h>
#include <bootloaders/boot.h>

ros::NodeHandle  nh;
const char * topic    ="toto";
ZSharpIR mycaptor(1, ZSharpIR::GP2Y0A21YK0F);

void setup() {
 nh.initNode();
 mycaptor.setup( &nh,   topic);
}

void loop() {
  
  mycaptor.loop();
  nh.spinOnce();
}
