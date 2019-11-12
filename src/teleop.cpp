#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class TeleopTurtle 
{
public:
  TeleopTurtle(ros::NodeHandle nh);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopTurtle::TeleopTurtle(ros::NodeHandle nh):
  nh_(nh),
  linear_(1),
  angular_(0)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, 2.60);
  nh_.param("scale_linear", l_scale_, 2.5);
  ROS_INFO("Ascale = %f",a_scale_);
  ROS_INFO("Lscale = %f",l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  twist.angular.z = a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh;
  TeleopTurtle teleop_turtle(nh);

  ros::spin();
}

