/***
 * Test the model of the NAO robot
 *
 * To use it:
 *     1) If floating base is not used (fbase = False):
 *               roslaunch nao_oscr display.launch
 *        If floating base is used (fbase = True):
 *               roslaunch nao_oscr display_floating.launch
 *     2) rosrun nao_oscr naoModel
 *
 */


#include <ros/ros.h>
#include <ros/package.h>

// Possible robot model backends (only 1 will be used)
#include <oscr/model/robot-model-pin.hpp>
#include <oscr/model/robot-model-rbdl.hpp>

#include <oscr/tools/model-utils.hpp>

#include <nao_oscr/markers.hpp>
#include <nao_oscr/joint-state-pub.hpp>


int main(int argc, char **argv)
{
  // Load the urdf model
  // ********************************************************************
  // Whether the floating base is used or not
  bool has_floating_base = false;
  // Path to the robot URDF (reduced with only 26 dofs)
  std::string model_pkg = ros::package::getPath("nao_oscr");
  std::string model_name = model_pkg + "/urdf/naoV40red.urdf";
  // Load the robot model: use RobotModelPin or RobotModelRbdl
  oscr::RobotModel* robot = new oscr::RobotModelPin(model_name,
                                                    has_floating_base);
  // ********************************************************************

  // Print model information
  oscr::printModelInfo(robot);

  // Important model information: joints and links
  std::vector<std::string> jnames = robot->jointNames();
  std::map<std::string, unsigned int> mjoint =
    oscr::mapJointNames(jnames, has_floating_base);
  std::map<std::string, unsigned int> mlink = robot->mapLinkNamesIDs();

  // Initialize ROS
  ros::init(argc, argv, "naoModel");
  ros::NodeHandle nh;

  // Joint Configuration
  Eigen::VectorXd q(robot->ndof());
  if (has_floating_base)
  {
    q <<
      0.0, 0.0, 0.33046, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
      1.15,  0.10, -1.4, -0.79, 0.0, 0.0,
      0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
      1.15, -0.10,  1.4,  0.79, 0.0, 0.0;
    if (false)
    {
      // Change base position
      q(0) = 0.2;
      q(1) = 0.4;
      q(2) = 0.3;
      // Change base orientation
      double t = M_PI/3.0;
      q(3) = cos(t);
      q(4) = sin(t);
    }
  }
  else
    q.setZero();
  robot->updateJointConfig(q);

  // Set feet on the ground (to be done just once)
  // double dsole = 0.04511;   // From URDF: dist between ankle and sole
  // q = robot->setFeetOnGround("r_ankle", "l_ankle", dsole);
  // robot->updateJointConfig(q);

  // Positions
  Eigen::VectorXd position1, position2;
  position1 = robot->linkPosition(mlink["l_wrist"]);
  position2 = robot->linkPosition(mlink["r_wrist"]);
  std::cout << "l_wrist position: " << position1.transpose() << std::endl;
  std::cout << "r_wrist position: " << position2.transpose() << std::endl;

  // Poses
  Eigen::VectorXd pose1, pose2;
  pose1 = robot->linkPose(mlink["l_wrist"]);
  pose2 = robot->linkPose(mlink["r_wrist"]);
  std::cout << "l_wrist pose: " << pose1.transpose() << std::endl;
  std::cout << "r_wrist pose: " << pose2.transpose() << std::endl;

  // Ball Markers
  std::vector<std::string> marker_links;
  std::vector<std::string> colors;
  marker_links.push_back("l_wrist"); colors.push_back("RED");
  marker_links.push_back("r_wrist"); colors.push_back("RED");
  RobotBallMarkers rball_markers(robot, nh, marker_links, colors);
  // Frame Markers
  RobotFrameMarkers rframe_markers(robot, nh, marker_links);

  // Publisher of  Joint States (to be parsed by rviz)
  JointStatePub jstate_pub(nh, robot->ndof(), has_floating_base);
  jstate_pub.setJointNames(jnames);
  jstate_pub.publish(q);

  // Sampling time
  unsigned int f = 100;
  double dt = static_cast<double>(1.0/f);

  ros::Rate rate(f); // Hz
  // while(ros::ok())
  for (unsigned int k=0; k<200; ++k)
  {
    q[mjoint["LShoulderPitch"]] += 0.005;
    q[mjoint["LShoulderRoll"]]  += 0.005;
    q[mjoint["LElbowRoll"]]     -= 0.005;
    q[mjoint["RShoulderPitch"]] += 0.005;
    q[mjoint["RShoulderRoll"]]  -= 0.005;
    q[mjoint["RElbowRoll"]]     += 0.005;

    robot->updateJointConfig(q);
    jstate_pub.publish(robot->getJointConfig());

    rball_markers.update();
    rframe_markers.update();

    rate.sleep();
  }

  return 0;
}
