#ifndef NAO_OSCR_MARKER_UTILS_HPP
#define NAO_OSCR_MARKER_UTILS_HPP

#include <nao_oscr/ball-marker.hpp>
#include <nao_oscr/frame-marker.hpp>

#include <oscr/model/robot-model.hpp>
#include <oscr/ik/kine-task.hpp>


/**
 * Class that helps showing ball markers at operational points (or links)
 *
 */
class RobotBallMarkers
{
public:
  /**
   * Constructor.
   * @param[in] robot oscr robot model
   * @param[in] nh ROS node handle
   * @param[in] links links where ball markers will be shown
   * @param[in] colors color for the ball marker (RED, BLUE, GREEN, LIGHTGRAY)

   */
  RobotBallMarkers(oscr::RobotModel* robot,
                   ros::NodeHandle& nh,
                   std::vector<std::string> links,
                   std::vector<std::string> colors);

  /**
   * Update the ball marker position by internally getting the robot link
   * position
   */
  void update();

private:
  /// Robot model
  oscr::RobotModel *robot_;
  /// Ball markers
  std::vector<Marker*> marker_;
  /// Link names
  std::vector<std::string> link_;
};


/**
 * Class that helps showing frame markers at operational points (or links)
 *
 */
class RobotFrameMarkers
{
public:
  /**
   * Constructor.
   * @param[in] robot oscr robot model
   * @param[in] nh ROS node handle
   * @param[in] links links where ball markers will be shown
   */
  RobotFrameMarkers(oscr::RobotModel* robot,
                    ros::NodeHandle& nh,
                    std::vector<std::string> links);

  /**
   * Update the ball marker position by internally getting the robot link
   * position
   */
  void update();

private:
  /// Robot model
  oscr::RobotModel *robot_;
  /// Frame markers
  std::vector<Marker*> marker_;
  /// Link names
  std::vector<std::string> link_;
};


/**
 * Class that helps showing markers for the desired and current position/pose
 * of a kinematic pose task
 *
 */
class TaskMarkers
{
public:
  /**
   * Constructor.
   * @param[in] nh ROS node handle
   */
  TaskMarkers(ros::NodeHandle& nh);

  /**
   * Add a task for which a marker will be used (for current and desired
   * position/pose). The desired position must be set in the task before
   * calling this method.
   *
   * @param[in] task task for which markers will be created
   */
  // void add(oscr::KineTaskPose* task);
  void add(oscr::KineTask* task);

  /**
   * Update the markers position/pose by internally getting the task current
   * position (the desired position/pose is assumed to be set before using the
   * add() method.
   */
  void update();

private:
  // Tasks
  std::vector<oscr::KineTask*> task_;
  /// Markers
  std::vector<Marker*> desired_marker_;
  std::vector<Marker*> current_marker_;
  // Pointer to the ros node handle
  ros::NodeHandle* nh_;
};

#endif
