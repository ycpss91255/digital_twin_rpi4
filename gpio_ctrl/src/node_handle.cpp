#include "gpio_ctrl/node_handle.h"

MotorNodeHandle::MotorNodeHandle(int argc, char** argv, string RobotName,
                                 uint64_t WheelNum) {
  string NodeName = "real_wheel" + to_string(WheelNum);
  this->WheelNS = "/real" + RobotName + "/wheel" + to_string(WheelNum);
  ros::init(argc, argv, NodeName);
  init();
}

MotorNodeHandle::~MotorNodeHandle() {
  ros::shutdown();
  // printf("\n\n%s shutdown\n\n", this->node_name.c_str());
}
// FIXME : change to ros param
void MotorNodeHandle::init() {
  this->n = new ros::NodeHandle();

  MotorFB_pub =
      n->advertise<std_msgs::Float64>((this->WheelNS + "/motorFB"), 100);
  MotorPos_sub = n->subscribe<std_msgs::Float64>(
      (this->WheelNS + "/cmd_pos"), 1, &MotorNodeHandle::CmdPosBack, this);
}

vector<int> MotorNodeHandle::getPin() {
  vector<int> PinData(7);
  vector<string> PinOrder = {"stp_sw", "dir", "pwm", "cs", "ena", "enb", "slp"};
  string WheelPin = this->WheelNS + "/pin/";

  n->getParam((this->WheelNS + "/slp_sw"), PinData.at(0));
  for (int i = 1; i < PinOrder.size(); i++)
    if (i != PinOrder.size() || PinData.at(0))
      n->getParam((WheelPin + PinOrder.at(i)), PinData.at(i));
  return PinData;
}

void MotorNodeHandle::CmdPosBack(const std_msgs::Float64::ConstPtr& msg) {
  CmdPos = msg->data;
}

void MotorNodeHandle::pubMotorFB(float feedback) {
  std_msgs::Float64 msg;
  msg.data = feedback;
  MotorFB_pub.publish(msg);
}
