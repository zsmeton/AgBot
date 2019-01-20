// license removed for brevity
#include <ros.h>
// Include message header files
#include <std_msgs/String.h>

/* {Node Description}
*/

// This node is a TEMPLATE

// Create any pin objects
const int ledPin = 13;

// Create the node handle which allows creation
// of publishers and subscribers and does serial
// communication
ros::NodeHandle  nh;

// Create message object(s) for pub/sub 
std_msgs::String {topic short hand}_msg;

// Create publisher(s)
ros::Publisher pub_{topic short hand}("{topic}", &{topic short hand}_msg);

// Create call back functions for subscriber(s)
void {topic short hand}_callback(const std_msgs::String& {topic short hand}_msg){
  if({topic short hand}.data == "TEST"){
    digitalWrite(ledPin, HIGH);
  }else{
    digitalWrite(ledPin, HIGH);
  }
}

// Create subsciber(s)
ros::Subscriber<std_msgs::Bool> sub_{topic short hand}("{topic}", &{topic short hand}_callback);

void setup(){
  // Set mode of pins
  pinMode(ledPin, OUTPUT);
  
  // Initialize the arduino node
  nh.initNode();

  // Advertise topic(s)
  nh.advertise(pub_{topic short hand});

  // Subscribe to topic(s)
  nh.subscribe(sub_{topic short hand});
}

void loop(){
  str_msg = "TEST";
  pub_{topic short hand}.publish(&str_msg);
  nh.spinOnce();
  delay(1000); // 1HZish
}