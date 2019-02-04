// ttyUSB1
// license removed for brevity
#include <ros.h>
// Include message header files
#include <std_msgs/String.h>

/* [motion_arduino] uses (drive_wheel_encoder) <encoder> and 
*  (wheel_cmd_speed) <wheel> to adjust each drive motors speed, 
*  publishes robots position changes through (robot_pos_delta) <position>
*/

// This node is a TEMPLATE

// Create any pin objects
const int ledPin = 13;

// Create the node handle which allows creation
// of publishers and subscribers and does serial
// communication
ros::NodeHandle  nh;

// Create message object(s) for pub/sub 
std_msgs::String position_msg;

// Create publisher(s)
ros::Publisher pub_position("robot_pos_delta", &position_msg);

// Create call back functions for subscriber(s)
void encoder_callback(const std_msgs::String& encoder_msg){
  if(encoder_msg.data == "TEST"){
    digitalWrite(ledPin, HIGH);
  }else{
    digitalWrite(ledPin, HIGH);
  }
}

void wheel_callback(const std_msgs::String& wheel_msg){
  if(wheel_msg.data == "TEST"){
    digitalWrite(ledPin, HIGH);
  }else{
    digitalWrite(ledPin, HIGH);
  }
}

// Create subsciber(s)
ros::Subscriber<std_msgs::String> sub_encoder("drive_wheel_encoder", &encoder_callback);
ros::Subscriber<std_msgs::String> sub_wheel("wheel_cmd_speed", &wheel_callback);

void setup(){
  // Set mode of pins
  pinMode(ledPin, OUTPUT);
  
  // Initialize the arduino node
  nh.initNode();

  // Advertise topic(s)
  nh.advertise(pub_position);

  // Subscribe to topic(s)
  nh.subscribe(sub_encoder);
  nh.subscribe(sub_wheel);
}

void loop(){
  position_msg.data = "TEST";
  pub_position.publish(&position_msg);
  nh.spinOnce();
  delay(1000); // 1HZish
}
