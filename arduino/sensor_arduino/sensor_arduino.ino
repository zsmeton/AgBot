// license removed for brevity
#include <ros.h>
// Include message header files
#include <std_msgs/String.h>

/* [sensor_arduino] pulls bumper button state and publish 
*  to (bumper_button) <bump>, pulls safety button state and 
*  publishes to (safety_button) <safety>, pulls wheel 
*  encoders and published to (drive_wheel_encoder) <encoder>, 
*  pulls compass data and published to (compass) <compass>, 
*  pulls gps info and publishes to (gps) <gps>
*/

// This node is a TEMPLATE

// Create any pin objects
const int ledPin = 13;

// Create the node handle which allows creation
// of publishers and subscribers and does serial
// communication
ros::NodeHandle  nh;

// Create message object(s) for pub/sub 
std_msgs::String bump_msg;
std_msgs::String safety_msg;
std_msgs::String encoder_msg;
std_msgs::String compass_msg;
std_msgs::String gps_msg;

// Create publisher(s)
ros::Publisher pub_bump("bumper_button", &bump_msg);
ros::Publisher pub_safety("safety_button", &safety_msg);
ros::Publisher pub_encoder("drive_wheel_encoder", &encoder_msg);
ros::Publisher pub_compass("compass", &compass_msg);
ros::Publisher pub_gps("gps", &gps_msg);

void setup(){
  // Set mode of pins
  pinMode(ledPin, OUTPUT);
  
  // Initialize the arduino node
  nh.initNode();

  // Advertise topic(s)
  nh.advertise(pub_bump);
  nh.advertise(pub_safety);
  nh.advertise(pub_encoder);
  nh.advertise(pub_compass);
  nh.advertise(pub_gps);
}

void loop(){
  bump_msg.data = "TEST";
  pub_bump.publish(&bump_msg);

  safety_msg.data = "TEST";
  pub_safety.publish(&safety_msg);
  
  encoder_msg.data = "TEST";
  pub_encoder.publish(&encoder_msg);
  
  compass_msg.data = "TEST";
  pub_compass.publish(&compass_msg);
  
  gps_msg.data = "TEST";
  pub_gps.publish(&gps_msg);
  
  nh.spinOnce();
  delay(1000); // 1HZish
}