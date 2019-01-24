// license removed for brevity
#include <ros.h>
// Include message header files
#include <std_msgs/String.h>

/* [sprayer_arduino] uses (weed_sprayer_teleop),  
*  (weed_sprayer_trigger), and (crop_sprayer_trigger) to move and 
*  activate the sprayers. * may add liquid level tracking *
*/

// This node is a TEMPLATE

// Create any pin objects
const int ledPin = 13;

// Create the node handle which allows creation
// of publishers and subscribers and does serial
// communication
ros::NodeHandle  nh;


// Create call back functions for subscriber(s)
void weed_teleop_callback(const std_msgs::String& weed_teleop_msg){
  if(weed_teleop_msg.data == "TEST"){
    digitalWrite(ledPin, HIGH);
  }else{
    digitalWrite(ledPin, HIGH);
  }
}

// Create call back functions for subscriber(s)
void weed_trigger_callback(const std_msgs::String& weed_trigger_msg){
  if(weed_trigger_msg.data == "TEST"){
    digitalWrite(ledPin, HIGH);
  }else{
    digitalWrite(ledPin, HIGH);
  }
}

// Create call back functions for subscriber(s)
void crop_trigger_callback(const std_msgs::String& crop_trigger_msg){
  if(crop_trigger_msg.data == "TEST"){
    digitalWrite(ledPin, HIGH);
  }else{
    digitalWrite(ledPin, HIGH);
  }
}

// Create subsciber(s)
ros::Subscriber<std_msgs::String> sub_weed_teleop("weed_sprayer_teleop", &weed_teleop_callback);
ros::Subscriber<std_msgs::String> sub_weed_trigger("weed_sprayer_trigger", &weed_trigger_callback);
ros::Subscriber<std_msgs::String> sub_crop_trigger("crop_sprayer_trigger", &crop_trigger_callback);

void setup(){
  // Set mode of pins
  pinMode(ledPin, OUTPUT);
  
  // Initialize the arduino node
  nh.initNode();

  // Subscribe to topic(s)
  nh.subscribe(sub_weed_teleop);
  nh.subscribe(sub_weed_trigger);
  nh.subscribe(sub_crop_trigger);
}

void loop(){
  nh.spinOnce();
  delay(1000); // 1HZish
}
