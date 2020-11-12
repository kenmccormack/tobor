#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#define LINEAR_AXIS 1
#define ANGULAR_AXIS 0 

class FilterVel
{
public:
  FilterVel(){ this->length = 10;}

  void setLength(double filter_length) {
     this->length = filter_length;
  }

  double filter(double velocity){
     block.push_back(velocity);
     if (this->block.size() > this->length ){
        this->block.erase(this->block.begin());
     }
     double sum = 0.0; 
     for (int ii=0; ii< this->length; ii++) {
	   sum = sum + this->block[ii];
     }
     return sum / this->length;
  };

private:
  int length;
  std::vector<double> block;
};


class TeleOpRobot
{
public:
  TeleOpRobot();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  FilterVel lfilter;      
  FilterVel afilter;

  ros::NodeHandle nh_;

  double l_scale_, a_scale_;
  double mixing_gain_; 
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

TeleOpRobot::TeleOpRobot()
{
  double angular_length, linear_length;
  nh_.getParam("teleop/scale/linear", a_scale_); 
  nh_.getParam("teleop/scale/linear", l_scale_); 
  nh_.getParam("teleop/mixing_gain", mixing_gain_);

  nh_.getParam("teleop/filter_length/linear", linear_length);
  nh_.getParam("teleop/filter_length/angular", angular_length);
  lfilter.setLength(linear_length);
  afilter.setLength(angular_length);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleOpRobot::joyCallback, this);

}

void TeleOpRobot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  double linear = l_scale_ * joy->axes[LINEAR_AXIS];

  /* add a boost to rotation when linear is small*/
  double aboost = ( 1.0 - abs(joy->axes[LINEAR_AXIS]) ) * mixing_gain_;
  double angular = (1.0 + aboost ) * (a_scale_ * joy->axes[ANGULAR_AXIS]);

  /* Filter the angular and linear components */ 
  angular = afilter.filter(angular);
  linear = lfilter.filter(linear);

  /* publish  a twist  control to motor driver */
  twist.angular.z = angular;
  twist.linear.x = linear;
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_robot");
  TeleOpRobot teleop_robot;

  ros::spin();
}

