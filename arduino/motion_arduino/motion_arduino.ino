// ttyUSB1
// license removed for brevity
#include <ros.h>
// Include message header files
#include <std_msgs/Float32MultiArray.h>

/* [motion_arduino] uses (drive_wheel_encoder) <encoder> and 
*  (wheel_cmd_speed) <wheel> to adjust each drive motors speed, 
*  publishes robots position changes through (robot_pos_delta) <position>
*/

// This node is a TEMPLATE

// Create any pin objects
int leftFrontMotor = 13;
int leftRearMotor = 12;
int rightFrontMotor = 11;
int rightRearMotor = 10;
int leftPWM = 122;
int rightPWM = 122;

// Create the node handle which allows creation
// of publishers and subscribers and does serial
// communication
ros::NodeHandle  nh;

// Create subsciber(s)
ros::Subscriber<std_msgs::Float32MultiArray> sub_wheel("wheel_cmd_speed", &wheel_callback);

void wheel_callback(const std_msgs::Float32MultiArray& wheel_msg){
  leftPWM = (int) 255*wheel_msg.data[0];
  rightPWM = (int) 255*wheel_msg.data[1];
}

void getROSParameters() {
  while (!nh.connected()) {
    nh.spinOnce();
  }
  int leftFront;
  int leftRear;
  int rightFront;
  int rightRear;

  if (nh.getParam("~left_front", &leftFront)) {
    leftFrontMotor = leftFront;
  }
  if (nh.getParam("~left_rear", &leftRear)) {
    leftRearMotor = leftRear;
  }
  if (nh.getParam("~right_front", &rightFront)) {
    rightFrontMotor = rightFront;
  }
  if (nh.getParam("~right_rear", &rightRear)) {
    rightRearMotor = rightRear;
  }
}


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
  nh.spinOnce();
  analogWrite(leftFrontMotor, leftPWM);
  analogWrite(leftRearMotor, leftPWM);
  analogWrite(rightFrontMotor, rightPWM);
  analogWrite(rightRearMotor, rightPWM);
}
