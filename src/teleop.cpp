#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class FilterVel
{
public:
  FilterVel(){ this->length = 10;}

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


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  FilterVel lfilter;      
  FilterVel afilter;

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(0)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, 2.20);
  nh_.param("scale_linear", l_scale_, 5.0);

   
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}

void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  double angular = a_scale_*joy->axes[angular_];
  double linear = l_scale_*joy->axes[linear_];
  /* add a boost to rotation when linear is small*/
  double aboost = ( 1.0 - linear/l_scale_ ) * 0.6;
  angular = (1.0 + aboost ) * angular;
  angular = afilter.filter(angular);
  linear = lfilter.filter(linear);
  twist.angular.z = angular;
  twist.linear.x = linear;
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  ros::spin();
}

