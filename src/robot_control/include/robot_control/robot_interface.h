#pragma once
/*
#include <gtest/gtest.h>

#include <memory>

#include "control_toolbox/pid.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/utilities.hpp"
*/
#include "std_msgs/Float64.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <std_msgs/Int32.h>

class RobotInterface: public hardware_interface::RobotHW //,public control_toolbox::Pid
{

    public:

        RobotInterface(ros::NodeHandle&);

        void update(const ros::TimerEvent& e);    
        void read(ros::Duration &period);
        void write(const ros::Duration& period);
        double ticksToAngle(const int &ticks);
        void rightEncoderTicksCallback(const std_msgs::Float64::ConstPtr& msg);
        void leftEncoderTicksCallback(const std_msgs::Float64::ConstPtr& msg);
    
    private:

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Duration elapsed_time_;
        ros::Duration update_freq_;
        ros::Timer looper_;
        ros::Time time_;
        ros::Publisher hardware_pub_;
        
        ros::Publisher pub_left_motor_value_;
        ros::Publisher pub_right_motor_value_;
        
        ros::Publisher pub_left_motor;
        ros::Publisher pub_right_motor;
        
        ros::Subscriber sub_left_encoder_ticks_;
        ros::Subscriber sub_right_encoder_ticks_;
        int encoder_ticks_[2];
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;

        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        
        std::vector<double> cmd_;
        std::vector<double> pos_;
        std::vector<double> vel_;
        std::vector<double> eff_;
        const double P = 1.0;
        const double I = 2.0;
        const double D = 3.0;
        const double I_MAX = 10.0;
        const double I_MIN = -10.0;
        const bool ANTIWINDUP = true;

        std::vector<std::string> names_;
};
