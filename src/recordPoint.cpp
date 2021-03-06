#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "TrajectData.h"
#include "memory"
#include "chrono"
#include <sys/stat.h>
#include <errno.h>
std::vector<trajectory_msgs::JointTrajectoryPoint> JointVector;
bool updateFalg = false;


namespace light
{
int mkpath(std::string s,mode_t mode=0755)
{
    size_t pre=0,pos;
    std::string dir;
    int mdret;

    if(s[s.size()-1]!='/'){
        // force trailing / so we can handle everything in loop
        s+='/';
    }

    while((pos=s.find_first_of('/',pre))!=std::string::npos){
        dir=s.substr(0,pos++);
        pre=pos;
        if(dir.size()==0) continue; // if leading / first time is 0 length
        if((mdret=::mkdir(dir.c_str(),mode)) && errno!=EEXIST){
            return mdret;
        }
    }
    return mdret;
}
}


void jointstatesCallback(const trajectory_msgs::JointTrajectoryConstPtr& msg)
{
  float pos[3],vel[3];
//  msg->joint_names;
//  msg->points.size();
  ROS_INFO("get the RobotTrajectoryConstPtr");
  pos[0]=msg->points[0].positions[0];
  pos[1]=msg->points[0].positions[1];
  pos[2]=msg->points[0].positions[2];
  vel[0]=msg->points[0].velocities[0];
  vel[1]=msg->points[0].velocities[1];
  vel[2]=msg->points[0].velocities[2];

  JointVector = msg->points;

  ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],vel[0],vel[1],vel[2]);
  updateFalg = true;
}

void trajectMessageToTrajectStructoin(std::vector<trajectory_msgs::JointTrajectoryPoint> &JointV,std::shared_ptr<TrajectGroupData> &trajectGroupData ){
    for(auto its : JointV){
        std::vector<double> pData = its.positions;
        std::vector<double> vData = its.velocities;
        std::vector<double> aData = its.accelerations;
        double time = its.time_from_start.toSec();

        trajectGroupData->setTrajectGroupElementData(pData, vData, aData, time);
    }

    ROS_WARN("setTrajectGroupElementData finish");


    std::shared_ptr<TrajectDataIoManager> trajectManager = std::make_shared<TrajectDataIoManager>(trajectGroupData);

    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //转为字符串
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S");
    std::string str_time = ss.str();

    light::mkpath(str_time);
    trajectManager->saveData(str_time+"/");
    ROS_WARN("trajectManager->saveData finish");
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "listener");
 
  ros::NodeHandle n;
  std::shared_ptr<TrajectGroupData> trajectGroupData = std::make_shared<TrajectGroupData>();

//  ros::Subscriber sub = n.subscribe("/fake_test/joint_states", 1, jointstatesCallback);
  ros::Subscriber sub = n.subscribe("/joint_path_command", 1, jointstatesCallback);
  ROS_INFO("Start listener the /fake/joint_states JointTrajectory");
  ros::Rate loop_rate(1);
//  ros::AsyncSpinner(1);

  while(ros::ok()){

    if(updateFalg){
        ROS_INFO("get the lastest message from /fake/joint_states JointTrajectory");
        trajectMessageToTrajectStructoin(JointVector, trajectGroupData);
        updateFalg = false;
    }else{
        loop_rate.sleep();
        ROS_INFO("Sleeping waitfor /fake/joint_states JointTrajectory");
    }
    ros::spinOnce();
  }
  
  return 0;
}


/*
 *
 *
  ros::Publisher robotTrajectortPub = node_handle.advertise<trajectory_msgs::JointTrajectory>("/fake_test/joint_states",1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  robotTrajectortPub.publish(my_plan.trajectory_.joint_trajectory);
*/

