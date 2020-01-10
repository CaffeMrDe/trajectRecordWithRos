#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "TrajectData.h"
#include "memory"
//std::vector<trajectory_msgs::JointTrajectoryPoint> JointVector;

int main(int argc, char **argv)
{
 
//  ros::init(argc, argv, "listener1");
 
//  ros::NodeHandle n;
  std::shared_ptr<TrajectGroupData> group;
  std::shared_ptr<TrajectDataIoManager> obj = std::make_shared<TrajectDataIoManager>();
  obj->readData();
  group = obj->getDataHandler();
  std::vector<double> dat = group->getAccelerationElement(1);
  dbg(dat.size());

  return 0;
}

