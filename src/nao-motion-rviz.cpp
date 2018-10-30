/***
 * Generate kinematic motion to control the position/orientation of some
 * operational points of the NAO robot. This program only visualizes the model
 * in RViz and does not send any command to the real robot.
 *
 * To use it:
 *
 *     1) If floating base is not used (fbase = False):
 *            roslaunch nao_oscr display.launch
 *        If floating base is used (fbase = True):
 *            roslaunch nao_oscr display-floating.launch
 *
 *     2) rosrun nao_oscr naoMotionRviz
 */


#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

// Possible robot model backends (only 1 will be used)
#include <oscr/model/robot-model-pin.hpp>
#include <oscr/model/robot-model-rbdl.hpp>

#include <oscr/ik/kine-task-pose.hpp>
#include <oscr/ik/osik-solvers.hpp>

#include <oscr/tools/model-utils.hpp>

#include <nao_oscr/joint-state-pub.hpp>
#include <nao_oscr/markers.hpp>


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

  // Important model information: joints and links
  std::vector<std::string> jnames = robot->jointNames();
  std::map<std::string, unsigned int> mlink = robot->mapLinkNamesIDs();
  // Joint limits (only needed for WQP and HQP)
  Eigen::VectorXd qmin, qmax, dqmax;
  qmin = robot->jointMinAngularLimits();
  qmax = robot->jointMaxAngularLimits();
  dqmax = robot->jointVelocityLimits();

  // Initialize ROS
  ros::init(argc, argv, "naoMotion");
  ros::NodeHandle nh;

  // Initial joint configuration
  Eigen::VectorXd q(robot->ndof());
  if (has_floating_base)
  {
    q << 0.0, 0.0, 0.33046, 1.0, 0.0, 0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
      1.15,  0.10, -1.4, -0.79, 0.0, 0.0,
      0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
      1.15, -0.10,  1.4,  0.79, 0.0, 0.0;
    if (false)
    {
      // Change base position
      q(0) = 0.2; q(1) = 0.4; q(2) = 0.3;
      // Change base orientation (quaternion: w, ex, ey, ez)
      double t = M_PI/3.0;
      q(3) = cos(t); q(4) = sin(t);
    }
  }
  else
  {
    q << 0.0, 0.0,
      0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
      1.15,  0.10, -1.4, -0.79, 0.0, 0.0,
      0.0, 0.0, -0.1, 0.3, -0.2, 0.0,
      1.15, -0.10,  1.4,  0.79, 0.0, 0.0;
  }
  robot->updateJointConfig(q);

  // Sampling time
  unsigned int f = 200;   // Frequency
  double dt = static_cast<double>(1.0/f);
  // Kinematic Solver
  oscr::OSIKSolverHQP solver(robot, q, dt);
  // oscr::OSIKSolverNS solver(robot, q, dt);
  // oscr::OSIKSolverWQP solver(robot, q, dt);
  solver.setJointLimits(qmin, qmax, dqmax);

  // Generic kinematic tasks
  oscr::KineTask *taskrh, *tasklh;
  // Vectors for positions/poses
  Eigen::VectorXd prWristCurr, prWristDes, plWristCurr, plWristDes;

  // Choose the option:
  //   * 0 = position both hands
  //   * 1 = pose both hands
  unsigned int choice = 1;

  if (choice == 0)
  {
    // Tasks
    taskrh = new oscr::KineTaskPose(robot, mlink["r_wrist"], "position");
    tasklh = new oscr::KineTaskPose(robot, mlink["l_wrist"], "position");
    taskrh->setGain(1.0);
    tasklh->setGain(1.0);

    // Desired value for right hand task
    taskrh->getSensedValue(prWristCurr);
    prWristDes = oscr::incPosition(prWristCurr, 0.02, -0.05, 0.10);
    taskrh->setDesiredValue(prWristDes);

    // Desired value for left hand task
    tasklh->getSensedValue(plWristCurr);
    plWristDes = oscr::incPosition(plWristCurr, 0.02, 0.05, 0.10);
    tasklh->setDesiredValue(plWristDes);
  }

  else if (choice == 1)
  {
    //KineTask* taskrh = new KineTaskPose(robot, RGRIPPER, "position");
    taskrh = new oscr::KineTaskPose(robot, mlink["r_wrist"], "pose");
    tasklh = new oscr::KineTaskPose(robot, mlink["l_wrist"], "pose");
    taskrh->setGain(1.0);
    tasklh->setGain(1.0);

    // Desired value for the right hand
    taskrh->getSensedValue(prWristCurr);
    prWristDes = oscr::incPoseLocal(prWristCurr, 0.05, -0.03, 0.08,
                                    -30.0, 1.0, 0.0, 0.0);
    taskrh->setDesiredValue(prWristDes);

    // Desired value for the left hand
    tasklh->getSensedValue(plWristCurr);
    plWristDes = oscr::incPoseLocal(plWristCurr, 0.05, 0.03, 0.08,
                                    30.0, 1.0, 0.0, 0.0);
    tasklh->setDesiredValue(plWristDes);
  }

  if (has_floating_base)
  {
    // Keep the feet on place
    oscr::KineTask *taskrf, *tasklf;
    taskrf = new oscr::KineTaskPose(robot, mlink["r_ankle"], "pose");
    tasklf = new oscr::KineTaskPose(robot, mlink["l_ankle"], "pose");
    taskrf->keep(10.0);
    tasklf->keep(10.0);
    solver.pushTask(taskrf);
    solver.pushTask(tasklf);
    // Keep the chest orientation
    oscr::KineTask *taskchest;
    taskchest = new oscr::KineTaskPose(robot, mlink["base_link"], "orientation");
    taskchest->keep(1.0);
    // solver.pushTask(taskchest); // Currently chest is not used
  }
  // Add the hand tasks
  solver.pushTask(taskrh);
  solver.pushTask(tasklh);

  // Markers
  TaskMarkers markers(nh);
  markers.add(tasklh);
  markers.add(taskrh);

  // Publisher of  Joint States (for rviz)
  JointStatePub jstate_pub(nh, robot->ndof(), has_floating_base);
  jstate_pub.setJointNames(jnames);
  jstate_pub.publish(q);

  // Log
  std::ofstream fileq;
  fileq.open("/tmp/q.dat");

  Eigen::VectorXd qdes;
  ros::Rate rate(f); // Hz
  while(ros::ok())
  {
    solver.getPositionControl(q, qdes);
    robot->updateJointConfig(q);
    jstate_pub.publish(robot->getJointConfig());

    markers.update();
    fileq << qdes.transpose() << std::endl;

    q = qdes;
    rate.sleep();
  }

  return 0;
}


