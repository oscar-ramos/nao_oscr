#include <nao_oscr/marker-utils.hpp>
#include <oscr/ik/kine-task-pose.hpp>


RobotBallMarkers::RobotBallMarkers(oscr::RobotModel* robot,
                                   ros::NodeHandle& nh,
                                   std::vector<std::string> links,
                                   std::vector<std::string> colors)
  :
  robot_(robot)
{
  if (links.size() != colors.size())
  {
    // TODO: Change to WARNING or ERROR
    ROS_INFO("Size of links and colors is different!");
    return;
  }
  for (unsigned int i=0; i<links.size(); ++i)
  {
    Marker *marker;
    // TODO: simplify this
    if (colors[i]=="RED")
      marker = new BallMarker(nh, RED);
    else if (colors[i]=="BLUE")
      marker = new BallMarker(nh, BLUE);
    else if (colors[i]=="GREEN")
      marker = new BallMarker(nh, GREEN);
    else if (colors[i]=="LIGHTGRAY")
      marker = new BallMarker(nh, LIGHTGRAY);
    else
    {
      // TODO: Change to WARNING or ERROR
      ROS_INFO("Color is invalid!");
      return;
    }
    marker_.push_back(marker);
    link_.push_back(links[i]);
  }
}


void RobotBallMarkers::update()
{
  for (unsigned int i=0; i<marker_.size(); ++i)
  {
    marker_[i]->setPose(robot_->linkPosition(link_[i]));
  }
}


RobotFrameMarkers::RobotFrameMarkers(oscr::RobotModel* robot,
                                     ros::NodeHandle& nh,
                                     std::vector<std::string> links)
  :
  robot_(robot)
{
  for (unsigned int i=0; i<links.size(); ++i)
  {
    Marker *marker = new FrameMarker(nh, 1.0);
    marker_.push_back(marker);
    link_.push_back(links[i]);
  }
}


void RobotFrameMarkers::update()
{
  for (unsigned int i=0; i<marker_.size(); ++i)
  {
    marker_[i]->setPose(robot_->linkPose(link_[i]));
  }
}


TaskMarkers::TaskMarkers(ros::NodeHandle& nh)
{
  nh_ = &nh;
}


void TaskMarkers::add(oscr::KineTask* task)
{
  task_.push_back(task);

  Eigen::VectorXd desired_pose;
  desired_pose = task->getDesiredValue();

  oscr::KineTaskPose* t = dynamic_cast<oscr::KineTaskPose*>(task);
  unsigned int task_type = t->getType();
  if (task_type == 0)
  {
    Marker *marker_curr = new BallMarker(*nh_, RED);
    Marker *marker_des = new BallMarker(*nh_, GREEN);
    marker_des->setPose(desired_pose);
    current_marker_.push_back(marker_curr);
    desired_marker_.push_back(marker_des);
  }
  if (task_type == 1)
  {
    std::cerr << "Currently no support for orientation tasks" << std::endl;
    return;
  }
  if (task_type == 2)
  {
    Marker *marker_curr = new FrameMarker(*nh_, 1.0);
    Marker *marker_des = new FrameMarker(*nh_, 1.0, 0.6);
    marker_des->setPose(desired_pose);
    current_marker_.push_back(marker_curr);
    desired_marker_.push_back(marker_des);
  }
}

void TaskMarkers::update()
{
  Eigen::VectorXd xsensed;
  for (unsigned int i=0; i<current_marker_.size(); ++i)
  {
    task_[i]->getSensedValue(xsensed);
    current_marker_[i]->setPose(xsensed);
    desired_marker_[i]->publish();
  }
}
