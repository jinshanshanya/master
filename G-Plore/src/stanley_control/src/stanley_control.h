#include <ros/network.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <boost/functional.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <ctime>
#include <iostream>
#include <thread>
#include<mutex>
#include<Eigen/Dense>
#include <algorithm>
#define pi 3.1415926

class Stanley_control{
    public:
    Stanley_control(ros::NodeHandle nh, ros::NodeHandle pri_nh);
    ~Stanley_control();
    
    void onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr & msg);
    void onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr & velocitymsg);
    float find_distance(Eigen::MatrixXd p1_F,Eigen::MatrixXd p2_F,Eigen::MatrixXd current_point_F );
    ros::Time timer_;
    void onTimer(const ros::TimerEvent & event);

    private:
    f2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    boost::optional<geometry_msgs::PoseStamped> current_pose_;
    ros::Subscriber sub_trajectory_;
    ros::Subscriber sub_current_velocity_;
    ros::Publisher pub_ctrl_cmd_;
    autoware_planning_msgs::Trajectory::ConstPtr trajectory_;
    geometry_msgs::TwistStamped::ConstPtr current_velocity_;

    private:
     float current_heading;
    std::vector<float> heading;
    int index_store;
    int index_current;
    float Lf;
    int setsize;
    float h_error;
    float v;
    float  steering_wheel_offset;
    float fd;
    float current_x;
    float current_y;

};