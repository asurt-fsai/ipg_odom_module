#include "ros/ros.h"
#include "vehiclecontrol_msgs/VehicleControl_msgs.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "sensor_msgs/Imu.h"
#include "PID.h"
#include "nav_msgs/Odometry.h"
#include "eufs_msgs/VehicleCommandsStamped.h"
#include "eufs_msgs/CanState.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#define PID_KP 0.5f
#define PID_KI 0.05f
#define PID_KD 0.015f
#define PID_TAU 0.02f
#define PID_LIM_MIN -1.0f
#define PID_LIM_MAX 1.0f
#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT 5.0f
#define SAMPLE_TIME_S 0.1f
#define WHEEL_RADIUS 0.253f


PIDController pid = { PID_KP, PID_KI, PID_KD,
                      PID_TAU,
                      PID_LIM_MIN, PID_LIM_MAX,
                      PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                      SAMPLE_TIME_S};

float Throttle = 0;
float Brake = 0;
float  velocity_setpoint = 0.0f;
float desired_steering_angle = 0;
float current_velocity = 0.0f;
float brake_set = 0.0f;
vehiclecontrol_msgs::VehicleControl_msgs Control_msg;

void Bridge_CB(const eufs_msgs::VehicleCommandsStamped &msg){
velocity_setpoint = msg.commands.rpm * M_PI * WHEEL_RADIUS / 30;
desired_steering_angle = msg.commands.steering * M_PI / 180;
}

void Bridge_Callback_state_estimation(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
{
 current_velocity = msg->twist.twist.linear.x;
}

void Bridge_Callback_state(const eufs_msgs::CanState &msg)
{
  ROS_INFO("State: %d", msg.as_state);
 if (msg.as_state != eufs_msgs::CanState::AS_DRIVING){
  brake_set = 1;
 }
  else{
    brake_set = 0;
  }
}



int main(int argc, char **argv)
{
ros::init(argc, argv, "IPG_Bridge");
ros::NodeHandle n;
ros::Publisher Bridge_pub;
ros::Subscriber Bridge_sub_control;
ros::Subscriber Bridge_sub_velocity;
ros::Subscriber Bridge_sub_state;

  ROS_INFO("Running");
  if (velocity_setpoint == 0){
    ROS_INFO("WE NOT GOIN TODAY");
  }
  Bridge_pub = n.advertise<vehiclecontrol_msgs::VehicleControl_msgs>("/carmaker/VehicleControl", 1000);
  Bridge_sub_control = n.subscribe("/state_machine/vehicle_commands", 1000, Bridge_CB);
  Bridge_sub_velocity = n.subscribe("/state_machine/twist", 1000, Bridge_Callback_state_estimation);
  Bridge_sub_state = n.subscribe("/state_machine/state", 1000, Bridge_Callback_state);
  ros::Rate loop_rate(10);
  while (ros::ok())
  { 
  Control_msg.use_vc = 1;
  PIDController_Update(&pid, velocity_setpoint, current_velocity);
    if (pid.out > 0)
    {
      Brake = 0;
      Throttle = pid.out;
      Control_msg.use_vc = 1;
      Control_msg.gas = Throttle;
      Control_msg.brake = Brake;
    }
    else
    {
      Throttle = 0;
      Brake = -1 * pid.out;
      Control_msg.use_vc = 1;
      Control_msg.gas = Throttle;
      Control_msg.brake = Brake;
    }
    Control_msg.steer_ang = desired_steering_angle * 6.66666666;
    if (brake_set == 1)
    {
      Control_msg.gas = 0;
      Control_msg.brake = 1;
    }

    Bridge_pub.publish(Control_msg);
    ROS_INFO("Publishing Vehicle control");
    loop_rate.sleep();
    ros::spinOnce();

  }
}

