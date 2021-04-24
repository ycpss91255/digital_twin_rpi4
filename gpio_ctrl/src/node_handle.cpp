#include "gpio_ctrl/node_handle.h"

MotorNodeHandle::MotorNodeHandle(int argc, char** argv, std::string RobotNS,
                                 std::string NodeName) {
  ros::init(argc, argv, NodeName);
  this->robot_ns = RobotNS;
  this->node_name = NodeName;
  init();
}

MotorNodeHandle::~MotorNodeHandle() {
  ros::shutdown();
  printf("\n\n%s shutdown\n\n", this->node_name.c_str());
}
// FIXME : change to ros param
void MotorNodeHandle::init() {
  this->n = new ros::NodeHandle();
  std::string MotorFBTopicName =
      this->robot_ns + "/" + this->node_name + "/motorFB";
  std::string MotorPosTopicName =
      this->robot_ns + "/" + this->node_name + "/cmd_pos";

  MotorFB_pub = n->advertise<std_msgs::Float64>(MotorFBTopicName, 100);
  MotorPos_sub = n->subscribe<std_msgs::Float64>(
      MotorPosTopicName, 1, &MotorNodeHandle::CmdPosBack, this);

#ifdef DEBUG
  printf("Motion_nodeHandle(DEBUG)\n");
#endif
}

void MotorNodeHandle::CmdPosBack(const std_msgs::Float64::ConstPtr& msg) {
  CmdPos = msg->data;
}

void MotorNodeHandle::pubMotorFB(float feedback) {
  std_msgs::Float64 msg;
  msg.data = feedback;
  MotorFB_pub.publish(msg);
}
