#include <nao_oscr/robot-interface.hpp>


RobotInterface::RobotInterface(ros::NodeHandle& nh,
                               const unsigned int& ndof_actuated,
                               const std::vector<std::string>& jnames)
  : ndof_actuated_(ndof_actuated),
    jsensed_msg_(new sensor_msgs::JointState)
{
  // assert(ndof_actuated_ == 26);

  nh_ = &nh;
  // To send joint angles to the robot
  pub_=nh_->advertise<naoqi_bridge_msgs::JointAnglesWithSpeed>("joint_angles",
                                                                1000);

  // To read joint configuration from the robot
  sub_=nh_->subscribe("joint_states", 1000,
                     &RobotInterface::callbackMeasuredJoint, this);

  // Message for the command to the joints
  jcmd_msg_.joint_names.resize(ndof_actuated_);
  jcmd_msg_.joint_angles.resize(ndof_actuated_);
  jcmd_msg_.speed = 0.2;
  for (unsigned int i=0; i<ndof_actuated_; ++i)
    jcmd_msg_.joint_names[i] = jnames[i];

}


void RobotInterface::callbackMeasuredJoint(const sensor_msgs::JointState::ConstPtr& msg)
{
  jsensed_msg_ = msg;
}


Eigen::VectorXd RobotInterface::getMeasuredJointPositions()
{
  // TODO: this should be maybe greater (e.g. 10?)
  ros::Rate iter_rate(1000);
  unsigned int ndof_sensed = jsensed_msg_->position.size();
  unsigned int niter=0, max_iter = 1e3;

  while (ndof_sensed != ndof_actuated_)
  {
    if (niter++ == max_iter)
    {
      std::cerr << "Sensed joint configuration does not have " << ndof_actuated_
                << "degrees of freedom, stopping ..." << std::endl;
      exit(0);
    }
    ndof_sensed = jsensed_msg_->position.size();
    ros::spinOnce();
    iter_rate.sleep();
  }

  // The vector qsensed must have 26 degrees of freedom for NAO
  Eigen::VectorXd qsensed = Eigen::VectorXd::Zero(ndof_actuated_);

  for (unsigned int i=0; i<ndof_actuated_; ++i)
  {
    // Map from the sensed values (naoqi format) to the oscr joint values order
    // (parsed with urdf) - This can be verified comparing the joint_states
    // message with the joints in the robot model (printModelInfo)
    qsensed[ naoqi2oscr[i] ] = jsensed_msg_->position[i];
  }

  return qsensed;
}


void RobotInterface::setJointPositionsCmd(const Eigen::VectorXd& q)
{
  for (unsigned int i=0; i<ndof_actuated_; ++i)
    jcmd_msg_.joint_angles[i] = q[i];

  jcmd_msg_.header.stamp = ros::Time::now();
  pub_.publish(jcmd_msg_);
}
