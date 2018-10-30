/***
 * Generate kinematic motion to control the position/orientation of some
 * operational points of the NAO robot
 *
 * To use it (with local naoqi):
 *     1) naoqi --verbose --broker-ip 127.0.0.1
 *     2) roslaunch nao_bringup nao_full_py.launch nao_ip:=127.0.0.1
 *     3) roslaunch nao_oscr display.launch
 *     4) rosrun nao_oscr naoMotionNaoqi
 *
 */


#include <ros/ros.h>
#include <ros/package.h>

#include <oscr/model/robot-model-pin.hpp>
#include <oscr/ik/kine-task-pose.hpp>
#include <oscr/ik/osik-solvers.hpp>

#include <oscr/tools/model-utils.hpp>

#include <nao_oscr/robot-interface.hpp>
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
  oscr::RobotModel* rmodel = new oscr::RobotModelPin(model_name,
                                                     has_floating_base);
  // ********************************************************************

  // Important model information: joints and links
  std::vector<std::string> jnames = rmodel->jointNames();
  std::map<std::string, unsigned int> mlink = rmodel->mapLinkNamesIDs();
  // Joint limits (only needed for WQP and HQP)
  Eigen::VectorXd qmin, qmax, dqmax;
  qmin = rmodel->jointMinAngularLimits();
  qmax = rmodel->jointMaxAngularLimits();
  dqmax = rmodel->jointVelocityLimits();

  // Initialize ROS
  ros::init(argc, argv, "naoMotionNaoqi");
  ros::NodeHandle nh;

  // Initialize interface oscr control - naoqi
  RobotInterface robot(nh, rmodel->ndofActuated(), jnames);

  // Read the initial joint configuration
  Eigen::VectorXd qsensed;
  qsensed = robot.getMeasuredJointPositions();
  rmodel->updateJointConfig(qsensed);

  // Sampling time
  unsigned int f = 100;   // Frequency
  double dt = static_cast<double>(1.0/f);

  // Kinematic solver
  //KineSolverWQP solver(rmodel, qsensed, dt);
  oscr::OSIKSolverHQP solver(rmodel, qsensed, dt);
  solver.setJointLimits(qmin, qmax, dqmax);

  // Choose the option:
  //   * 0 = position both hands
  //   * 1 = pose both hands
  unsigned int choice = 1;

  // Generic kinematic tasks
  oscr::KineTask *taskrh, *tasklh;
  // Vectors for positions/poses
  Eigen::VectorXd prWristCurr, prWristDes, plWristCurr, plWristDes;

  if (choice == 0)
  {
    // Tasks
    taskrh = new oscr::KineTaskPose(rmodel, mlink["r_wrist"], "position");
    tasklh = new oscr::KineTaskPose(rmodel, mlink["l_wrist"], "position");
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

    solver.pushTask(taskrh);
    solver.pushTask(tasklh);
  }
  else if (choice == 1)
  {
    taskrh = new oscr::KineTaskPose(rmodel, mlink["r_wrist"], "pose");
    tasklh = new oscr::KineTaskPose(rmodel, mlink["l_wrist"], "pose");
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

    solver.pushTask(tasklh);
    solver.pushTask(taskrh);
  }

  // Markers
  TaskMarkers markers(nh);
  markers.add(tasklh);
  markers.add(taskrh);

  Eigen::VectorXd qdes;
  ros::Rate rate(f); // Hz
  while(ros::ok())
  {
    solver.getPositionControl(qsensed, qdes);
    rmodel->updateJointConfig(qdes);

    robot.setJointPositionsCmd(qdes);
    markers.update();

    qsensed = qdes;
    rate.sleep();
  }

  return 0;
}


