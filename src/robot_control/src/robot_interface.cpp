#include "robot_control/robot_interface.h"
#include "std_msgs/Float64.h"


RobotInterface::RobotInterface(ros::NodeHandle& nh) : nh_(nh), 
            pnh_("~"),
            pos_(2, 0),
            vel_(2, 0),
            eff_(2, 0),
            cmd_(2, 0),
            names_{"left_wheel_rot_joint", "right_wheel_rot_joint"}
{
    // Read from the param server
  //  pnh_.param("joint_names", names_, names_);

    // Init the publisher with the hardware
        pub_left_motor_value_ = nh_.advertise<std_msgs::Float64>("/left_wheel/setpoint", 1);
        pub_right_motor_value_ = nh_.advertise<std_msgs::Float64>("/right_wheel/setpoint", 1);
      pub_left_motor = nh_.advertise<std_msgs::Float64>("/left_wheel/state", 1);
      pub_right_motor= nh_.advertise<std_msgs::Float64>("/right_wheel/state", 1);
        // Setup subscriber for the wheel encoders
        sub_left_encoder_ticks_ = nh_.subscribe("ticks_left", 1, &RobotInterface::leftEncoderTicksCallback, this);
        sub_right_encoder_ticks_ = nh_.subscribe("ticks_right", 1, &RobotInterface::rightEncoderTicksCallback, this);
    
    ROS_INFO("Starting Arduinobot Hardware Interface...");

     for (int i = 0; i < 2; i++) 
    {
    // Create joint state interface
    hardware_interface::JointStateHandle state_handle(names_.at(i), &pos_.at(i), &vel_.at(i), &eff_.at(i));
    joint_state_interface_.registerHandle(state_handle);

    // Create velocity joint interface
    hardware_interface::JointHandle velocity_handle(state_handle, &cmd_.at(i));
    velocity_joint_interface_.registerHandle(velocity_handle);

    // Create joint limits interface
    joint_limits_interface::JointLimits limits;
    joint_limits_interface::getJointLimits(names_.at(i), nh_, limits);
    joint_limits_interface::VelocityJointSaturationHandle joint_limits_handle( velocity_handle,limits);
    velocityJointSaturationInterface.registerHandle(joint_limits_handle);
   
    pos_.at(i) = 0;
    vel_.at(i) = 0;

    eff_.at(i) = 0;
    cmd_.at(i) = 0;
    }

// Register interfaces
registerInterface(&joint_state_interface_);
registerInterface(&velocity_joint_interface_);
registerInterface(&velocityJointSaturationInterface);

    ROS_INFO("Preparing the Controller Manager");
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    update_freq_ = ros::Duration(0.1);
    looper_ = nh_.createTimer(update_freq_, &RobotInterface::update, this);
    
    ROS_INFO("Ready to execute the control loop");

/* ***************************************************************************
            // Initialize the pid controllers for the motors using the robot namespace
            std::string pid_namespace = "pid/" + motor_names[i];
            ROS_INFO_STREAM("pid namespace: " << pid_namespace);
            ros::NodeHandle nh(root_nh, pid_namespace);
            // TODO implement builder pattern to initialize values otherwise it is hard to see which parameter is what.
            pids_[i].init(nh, 0.0, 10.0, 1.0, 1.0, 0.0, 0.0, false, -max_velocity_, max_velocity_);
            pids_[i].setOutputLimits(-max_velocity_, max_velocity_);
/* ***************************************************************************/
  /*  const int numControllers = 2;  // Adjust the number of controllers as needed
    std::vector<Pid> controllers(numControllers);
    double params1[] = {1.0, 2.0, 3.0, 10.0, -10.0, true};
    controllers.at(0).initPid(params1[0], params1[1], params1[2], params1[3], params1[4], params1[5]);

    // Parameters for the second PID controller
    double params2[] = {0.5, 1.0, 1.5, 5.0, -5.0, true};
    controllers.at(1).initPid(params2[0], params2[1], params2[2], params2[3], params2[4], params2[5]);
*/
}

void RobotInterface::update(const ros::TimerEvent& e)
{
    // This function is called periodically in order to update the controller
    // manager about the progress in the execution of the goal of the hardware
    ROS_INFO("Update Event");
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read(elapsed_time_);
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}
void RobotInterface::read(ros::Duration &period)
{
        ros::Duration elapsed_time = period;
        // Read from robot hw (motor encoders)
        // Fill joint_state_* members with read values
        double wheel_angles[2];
        double wheel_angle_deltas[2];
        for (std::size_t i = 0; i < 2; i++)
        {
            wheel_angles[i] = ticksToAngle(encoder_ticks_[i]);
            //double wheel_angle_normalized = normalizeAngle(wheel_angle);
            wheel_angle_deltas[i] = wheel_angles[i] - pos_.at(i);

            pos_.at(i) += wheel_angle_deltas[i];
            vel_.at(i) = wheel_angle_deltas[i] / period.toSec();
            eff_.at(i) = 0.0; // unused with diff_drive_controller
        }

}
void RobotInterface::write(const ros::Duration& period)
    {
        ros::Duration elapsed_time = period;
        // Write to robot hw
        // joint velocity commands from ros_control's RobotHW are in rad/s
        // Convert the velocity command to a percentage value for the motor
        // This maps the velocity to a percentage value which is used to apply
        // a percentage of the highest possible battery voltage to each motor.
        
        velocityJointSaturationInterface.enforceLimits(elapsed_time);   
        std_msgs::Float64 left_motor;
        std_msgs::Float64 right_motor;

        std_msgs::Float64 left_motor1;
        std_msgs::Float64 right_motor1;
        
        double error_left_motor=cmd_.at(0)-vel_.at(0);
        double error_right_motor =cmd_.at(1)-vel_.at(1);
   
        left_motor1.data=vel_.at(0);
        right_motor1.data=vel_.at(0);
        pub_left_motor.publish(left_motor1);
        pub_right_motor.publish(right_motor1);
   
   /*     
        double output_left = controllers.at(0).computeCommand(error_left_motor, period);
        
        double output_right = controllers.at(1).computeCommand(error_right_motor, period);
*/
      //  double output_right = pids_[1](vel_.at(1), cmd_.at(1), period);

       // left_motor.data = output_left / max_velocity_ * 100.0;
        //right_motor.data = output_right / max_velocity_ * 100.0;

        left_motor.data=cmd_.at(0);
        right_motor.data=cmd_.at(1);

    // Publish the PID computed motor commands to the left and right motors
        pub_left_motor_value_.publish(left_motor);
        pub_right_motor_value_.publish(right_motor);
    }
void RobotInterface::leftEncoderTicksCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        encoder_ticks_[0] = msg->data;
        ROS_DEBUG_STREAM_THROTTLE(1, "Left encoder ticks: " << msg->data);
    }

void RobotInterface::rightEncoderTicksCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        encoder_ticks_[1] = msg->data;
        ROS_DEBUG_STREAM_THROTTLE(1, "Right encoder ticks: " << msg->data);
    }

double RobotInterface::ticksToAngle(const int &ticks) 
    {
        // Convert number of encoder ticks to angle in radians
        double angle = (double)ticks * (2.0*M_PI / 542.0);
        ROS_DEBUG_STREAM_THROTTLE(1, ticks << " ticks correspond to an angle of " << angle);
    return angle;
    }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduinobot_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    RobotInterface robot(nh);

    // Keep ROS up and running
    spinner.spin();
    return 0;
}
