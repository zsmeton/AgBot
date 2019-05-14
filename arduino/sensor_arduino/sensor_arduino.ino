// ttyACM0
// license removed for brevity
#include <ros.h>
// Include message header files
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "UltrasonicSensor.h"

/* [sensor_arduino] pulls ultrasonic sensor state and publish
   to (ultra_detection) <ultra>, pulls safety button state and
   publishes to (safety_button) <safety>, pulls wheel
   encoders and published to (drive_wheel_encoder) <encoder>,
   pulls compass data and published to (compass) <compass>,
   pulls gps info and publishes to (gps) <gps>
*/

// This node is a TEMPLATE

// Create the node handle which allows creation
// of publishers and subscribers and does serial
// communication
ros::NodeHandle  nh;


// Get ROS Parameters:
float stopping_distance = 1.8288; // default to 6 meters


// Create message object(s) for pub/sub
std_msgs::Bool ultra_msg;
std_msgs::String safety_msg;
std_msgs::String encoder_msg;
std_msgs::String compass_msg;
std_msgs::String gps_msg;

// Create publisher(s)
ros::Publisher pub_ultra("ultra_detection", &ultra_msg);
ros::Publisher pub_safety("safety_button", &safety_msg);
ros::Publisher pub_encoder("drive_wheel_encoder", &encoder_msg);
ros::Publisher pub_compass("compass", &compass_msg);
ros::Publisher pub_gps("gps", &gps_msg);

// create sensor instances
UltrasonicSensor ultrasonic1 = UltrasonicSensor(7, 6, 2);
UltrasonicSensor ultrasonic2 = UltrasonicSensor(5, 4, 2);

void setup() {
  Serial.begin(9600);
  // Set mode of pins

  // Initialize the arduino node
  nh.initNode();

  // Advertise topic(s)
  nh.advertise(pub_ultra);
  nh.advertise(pub_safety);
  nh.advertise(pub_encoder);
  nh.advertise(pub_compass);
  nh.advertise(pub_gps);

  // Setup ros parameters
  getROSParameters();
}

void loop() {
  bool ultra = (ultrasonic1.obstruction() || ultrasonic1.obstruction());
  ultra_msg.data = ultra;
  pub_ultra.publish(&ultra_msg);

  safety_msg.data = "TEST";
  pub_safety.publish(&safety_msg);

  encoder_msg.data = "TEST";
  pub_encoder.publish(&encoder_msg);

  compass_msg.data = "TEST";
  pub_compass.publish(&compass_msg);

  gps_msg.data = "TEST";
  pub_gps.publish(&gps_msg);

  nh.spinOnce();
  delay(10); // 2HZish
}

void getROSParameters() {
  while (!nh.connected()) {
    nh.spinOnce();
  }
  int ultrasonic10;
  int ultrasonic11;
  int ultrasonic20;
  int ultrasonic21;
  int stop_dist;

  if (nh.getParam("~ultrasonic1_echo", &ultrasonic10)) {
    ultrasonic1.setEchoPin(ultrasonic10);
  }
  if (nh.getParam("~ultrasonic1_trig", &ultrasonic11)) {
    ultrasonic1.setTrigPin(ultrasonic11);
  }
  if (nh.getParam("~ultrasonic2_echo", &ultrasonic20)) {
    ultrasonic2.setEchoPin(ultrasonic20);
  }
  if (nh.getParam("~ultrasonic2_trig", &ultrasonic21)) {
    ultrasonic2.setTrigPin(ultrasonic21);
  }
  if (nh.getParam("~stopping_distance", &stop_dist, 1)) {
    ultrasonic1.setStopDist(stop_dist);
    ultrasonic2.setStopDist(stop_dist);
  }
}
