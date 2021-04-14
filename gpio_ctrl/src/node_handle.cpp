#include "gpio_ctrl/node_handle.h"

MotorNodeHandle::MotorNodeHandle(int argc, char **argv, std::string NodeName) {
  ros::init(argc, argv, NodeName,ros::init_options::AnonymousName);
  init();
}

MotorNodeHandle::~MotorNodeHandle() {
  ros::shutdown();

#ifdef DEBUG
  printf("~Motion_nodeHandle(DEBUG)\n");
#endif
}

void MotorNodeHandle::init() {
  this->n = new ros::NodeHandle();
  // MotorEnc_pub = n->advertise<motion::FourMotorStates>(motor_enc_topic_name, 1000);
  // MotorSpeed_pub = n->advertise<motion::FourMotorStates>(motor_speed_topic_name, 1000);
  // CmdVal_sub = n->subscribe<geometry_msgs::Twist>(
  //     motion_topic_name, 1000, &MotionNodeHandle::CmdVelBack, this);

#ifdef DEBUG
  printf("Motion_nodeHandle(DEBUG)\n");
#endif
}
