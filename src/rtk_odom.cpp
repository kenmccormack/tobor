#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <nav_msgs/Odometry.h>
#include <string>
#include <GeographicLib/Geodesic.hpp>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>






using namespace std;
using namespace GeographicLib;
using namespace boost;

const Geocentric& earth = Geocentric::WGS84();


class RTKOdom
{
public:
  RTKOdom(ros::NodeHandle nh);

private:
  
  ros::Subscriber _rtk_sub;
  ros::NodeHandle _nh;
  void rtk_callback(const std_msgs::String::ConstPtr& rtk_msg);

  typedef enum
  {
      MSG_TYPE,
      MSG_NUMBER,
      UTC_TIME,
      STATUS,
      LAT,
      NS_INDICATE,
      LON,
      EW_INDICATE,
      ALTITUDE,
      EAST_VEL,
      NORTH_VEL,
      UP_VEL,
      UTC_DATE,
      MODE,
      RTK_AGE,
      RTK_RATIO
  } PSTI30_MSG; 

  


  bool _initialized;

  ros::Publisher _odom_pub; 

  //Geocentric _earth(Constants::WGS84_a(), Constants::WGS84_f());
  LocalCartesian _proj;

  double convert_coordinate(int deg_digits, std::string coordinate)
  {
      double degrees = stod(coordinate.substr(0,deg_digits));
      double minutes = stod(coordinate.substr(deg_digits));

      double retval =  degrees + minutes / 60.0;

      return retval;


  }

};


RTKOdom::RTKOdom(ros::NodeHandle nh)  : _nh(nh)
{

      _rtk_sub = _nh.subscribe<std_msgs::String>("/rtk_gps", 10, &RTKOdom::rtk_callback, this);

      _odom_pub = _nh.advertise<nav_msgs::Odometry>("/gps_odom",10);

      _initialized  = false; 

      

  
}

void RTKOdom::rtk_callback(const std_msgs::String::ConstPtr& rtk_msg)
{
    //ROS_INFO("GPS MSG");
    //ROS_INFO("%s", rtk_msg->data.c_str());

    /*convert the string into it's pieces*/
    std::vector<std::string> msg_elements;
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tokens(rtk_msg->data, sep);
    BOOST_FOREACH (const std::string& t, tokens)
    {
      msg_elements.push_back(t);
    }

    //ROS_INFO("base : %s", msg_elements[4].c_str());
   // ROS_INFO("Status : %s", msg_elements[3].c_str());
    string status = msg_elements[STATUS];
    string mode  = msg_elements[MODE];
    
    if (msg_elements[LAT] != "nan")
    {
        double lat = convert_coordinate(2, msg_elements[LAT]) ;
        double lon = convert_coordinate(3, msg_elements[LON]) ; 
        double alt = std::stod(msg_elements[ALTITUDE]);
    
        double east_vel = std::stod(msg_elements[EAST_VEL]);
        double north_vel = std::stod(msg_elements[NORTH_VEL]);
        //ROS_INFO("%lf N, %lf W", lat, lon);

        if ( _initialized == false)
        {

        _proj.Reset( lat, lon, alt);
        _initialized = true;

        }
        else
        {
        // forward calculation
        double x, y, z;
        _proj.Forward(lat, lon, alt, x, y, z);

        double covariance;
        if (mode == "R")
        {
            covariance = .01;
        }
        else
        {
            covariance = .2;
        }


        //ROS_INFO("%s/%s, x: %f  y:%f  z:%f [%lf,%lf]", status.c_str(), mode.c_str(), x,y,z, east_vel, north_vel);

        //publish to odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now(); // rtk_msg->header.stamp;
        odom_msg.header.frame_id = "odom"; // rtk_msg->header.frame_id;
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;
        odom_msg.pose.pose.orientation.x = 1;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 0;
        odom_msg.pose.covariance = {covariance, 0, 0, 0, 0, 0,  // covariance on gps_x
                        0, covariance, 0, 0, 0, 0,  // covariance on gps_y
                        0, 0, covariance, 0, 0, 0,  // covariance on gps_z
                        0, 0, 0, 99999, 0, 0,  // large covariance on rot x
                        0, 0, 0, 0, 99999, 0,  // large covariance on rot y
                        0, 0, 0, 0, 0, 99999} ; // large covariance on rot z

        _odom_pub.publish(odom_msg);

                    

        
        
        
        }
    }
    


  


    /*publish odometry*/ 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtk_odom");
  ros::NodeHandle nh; 
  RTKOdom rtkod(nh);

  ros::spin();
}

