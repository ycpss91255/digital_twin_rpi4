#include "gpio_ctrl/node_handle.h"

MotorNodeHandle::MotorNodeHandle(int argc, char** argv, string RobotName,
                                 uint64_t WheelNum) {
  this->WheelNum = WheelNum;
  this->RobotName = RobotName;
  this->WheelNS = "/real" + RobotName + "/wheel" + to_string(WheelNum);

  string NodeName = "real_wheel" + to_string(WheelNum);
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
      this->n->advertise<std_msgs::Float64>((this->WheelNS + "/motorFB"), 100);
  MotorCmdFB_pub = this->n->advertise<gpio_ctrl::MotorCmdFB>(
      (this->WheelNS + "/motor_cmdFB"), 100);
  MotorPos_sub = this->n->subscribe<std_msgs::Float64>(
      (this->WheelNS + "/cmd_pos"), 1, &MotorNodeHandle::CmdPosBack, this);

  // Digital motor
  string DigitalMotorPosTopicName = "/Digital" + RobotName + "/wheel" +
                                    to_string(this->WheelNum) + "/motorFB";
  DigitalMotorPos_sub = this->n->subscribe<std_msgs::Float64>(
      DigitalMotorPosTopicName, 1, &MotorNodeHandle::DigitalMotorPosBack, this);
}

vector<int> MotorNodeHandle::getPin() {
  vector<int> PinData(7);
  vector<string> PinOrder = {"stp_sw", "dir", "pwm", "cs", "ena", "enb", "slp"};
  string WheelPin = this->WheelNS + "/pin/";
  bool slp_sw;
  this->n->getParam((this->WheelNS + "/slp_sw"), slp_sw);
  PinData.at(0) = (slp_sw ? 1 : 0);
  for (int i = 1; i < PinOrder.size(); i++)
    if (i != PinOrder.size() || PinData.at(0)) {
      this->n->getParam((WheelPin + PinOrder.at(i)), PinData.at(i));
    }
#ifdef DEBUG
  for (int i = 0; i < PinOrder.size(); i++) printf("%d\n", PinData.at(i));
#endif
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

void MotorNodeHandle::pubMotorCmdFB(gpio_ctrl::MotorCmdFB Cmd) {
  gpio_ctrl::MotorCmdFB msg;
  msg.dir = Cmd.dir;
  msg.pwm = Cmd.pwm;
  MotorCmdFB_pub.publish(msg);
}

float MotorNodeHandle::getCmdPos() { return this->CmdPos; }

float MotorNodeHandle::getDigitalCmdPos() { return this->DigitalCmdPos; }

void MotorNodeHandle::DigitalMotorPosBack(
    const std_msgs::Float64::ConstPtr& msg) {
  this->DigitalCmdPos = msg->data;
}
