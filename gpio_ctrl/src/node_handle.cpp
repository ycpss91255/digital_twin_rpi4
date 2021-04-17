#include "gpio_ctrl/node_handle.h"

MotorNodeHandle::MotorNodeHandle(int argc, char** argv, std::string NodeName) {
  ros::init(argc, argv, NodeName);
  this->node_name = NodeName;
  init(NodeName);
}

MotorNodeHandle::~MotorNodeHandle() {
  ros::shutdown();
  printf("\n\n%s shutdown\n\n", this->node_name.c_str());
}

void MotorNodeHandle::init(std::string NodeName) {
  this->n = new ros::NodeHandle();

  std::string MotorPosTopicName = NodeName + "/cmd_pos";
  std::string MotorFBTopicName = NodeName + "/motorFB";

  MotorFB_pub = n->advertise<std_msgs::Float32>(MotorFBTopicName, 100);
  MotorPos_sub = n->subscribe<std_msgs::Float32>(
      MotorPosTopicName, 100, &MotorNodeHandle::CmdPosBack, this);

#ifdef DEBUG
  printf("Motion_nodeHandle(DEBUG)\n");
#endif
}

void MotorNodeHandle::CmdPosBack(const std_msgs::Float32::ConstPtr& msg) {
  CmdPos = msg->data;
  printf("%f\n",msg->data);
}

void MotorNodeHandle::pubMotorFB(float feedback) {
  std_msgs::Float32 msg;
  msg.data = feedback;
  MotorFB_pub.publish(msg);
}
