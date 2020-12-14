// C library headers
#include <stdio.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include<iostream>
#include<vector>
#include<sstream>

#include "MadgwickAHRS.h"


class MPU9250Reader
{
public: 
    MPU9250Reader(std::string port_name, float sampleFrequency, std::string sensor_name, ros::NodeHandle nh): nh_(nh), serial_port_(-1)
    {
        // Setup diagnostics
        setup_diag();

        if (open_port(port_name))
        {
            madg_.begin(sampleFrequency); 
            imu_pub_ = nh.advertise<sensor_msgs::Imu>(sensor_name + std::string("/data"),100);
        }
        else
        {
            ROS_ERROR("Failed to open imu serial port");
        }
        
    }
    ~MPU9250Reader(){}

   
    bool scan()
    {
        if( serial_port_ == -1)
        {
            return false; 
        }
        else
        {
            read_port(read_buffer_);
            std::stringstream sample_str(read_buffer_);
            std::vector<float> data;
            std::vector<std::string> parsed;
            //std::cout << std::string(read_buffer_) << std::endl;
            while ( sample_str.good()) {
                std::string substr;
                getline(sample_str, substr, ',');
                data.push_back(stof(substr));
            }
           
            if (data.size() >= 9)
            {
                /*convert data. accell(xyz), gyro(xyz), mag(xyz)*/ 
                madg_.update(data.at(3), data.at(4), data.at(5),
                            data.at(0), data.at(1), data.at(2),
                            data.at(6), data.at(7), data.at(8));

                //std::cout << madg_.getRoll() << " " <<  madg_.getPitch() << " " << madg_.getYaw() << "    " << data.at(6) << " " << data.at(7) << std::endl;
            

                float w,x,y,z;
                madg_.getQuaternion(w,x,y,z);
                sensor_msgs::Imu imu_msg;
                std_msgs::Header header; 
                header.stamp = ros::Time::now();
                header.frame_id = std::string("imu");
                imu_msg.orientation.x = x;
                imu_msg.orientation.y = y;
                imu_msg.orientation.z = z;
                imu_msg.orientation.w = w;
                imu_msg.angular_velocity.x = data.at(3);
                imu_msg.angular_velocity.y = data.at(4);
                imu_msg.angular_velocity.z = data.at(5);
                imu_msg.linear_acceleration.x = data.at(0);
                imu_msg.linear_acceleration.y = data.at(1);
                imu_msg.linear_acceleration.z = data.at(2);

                /*out the door*/
                imu_pub_.publish(imu_msg);

                if (data.size()==11)
                {
                    rtk_radio_ = data.at(10);
                    imu_temp_ = data.at(9);
                }
            }
            else
            {
                ROS_WARN("Imu received fewer datapoints");
                return false;
            }
        }
        return true; 
    }

private:
    int serial_port_;
    char read_buffer_[1000];
    Madgwick madg_;
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Publisher diag_pub_;
    double last_diagnostics_; 
    ros::Timer diag_timer; 
    double rtk_radio_;
    double imu_temp_;
    
    void diagCallback(const ros::TimerEvent &)
    {
        diagnostic_msgs::DiagnosticArray diag_array_msg;
        diagnostic_msgs::DiagnosticStatus status;
        std_msgs::Header h1; 
        h1.stamp = ros::Time::now();
        diag_array_msg.header = h1;

        status.name=std::string("rtk");
        if (rtk_radio_ == 0.0)
        {
            status.message = std::string("No receiption");
            status.level = diagnostic_msgs::DiagnosticStatus::WARN; 
        } else {
            status.message = std::string("Data received");
            status.level = diagnostic_msgs::DiagnosticStatus::OK;
        }
        diag_array_msg.status.push_back(status);

        status.name = "Imu temp";
        status.message = std::string("Imu Temperature (C)");
        status.level = diagnostic_msgs::DiagnosticStatus::OK;
        diagnostic_msgs::KeyValue temp;
        temp.key="Temperature";
        temp.value =  std::to_string(imu_temp_);
        status.values.push_back(temp);
        diag_array_msg.status.push_back(status);

        diag_pub_.publish(diag_array_msg);
    }

    bool setup_diag() {
        diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics",100);
        diag_timer = nh_.createTimer(ros::Duration(1.0), &MPU9250Reader::diagCallback, this);
    }

    int read_port(char * response)
    {
        int n = 0,
        spot = 0;
        char buf = '\0';

        /* Whole response*/
        memset(response, '\0', sizeof response);

        do {
            n = read( serial_port_, &buf, 1 );
            sprintf( &response[spot], "%c", buf );
            spot += n;
        } while( buf != '\r' && n > 0);

        if (n < 0) {
            std::cout << "Error reading: " << strerror(errno) << std::endl;
            return 1;
        }
        else if (n == 0) {
            std::cout << "Read nothing!" << std::endl;
            return 1; 
        }
        else {
            return 0;
        }
    }
    

    bool open_port(std::string port_name)
    {
        /*https://stackoverflow.com/questions/18108932/reading-and-writing-to-serial-port-in-c-on-linux*/
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        serial_port_ = open(port_name.c_str(), O_RDWR| O_NOCTTY );

        /* Error Handling */
        if ( tcgetattr ( serial_port_, &tty ) != 0 ) {
            std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
            return false; 
        }

        /* Set Baud Rate */
        cfsetospeed (&tty, (speed_t)B115200);
        cfsetispeed (&tty, (speed_t)B115200);

        /* Setting other Port Stuff */
        tty.c_cflag     &=  ~PARENB;            // Make 8n1
        tty.c_cflag     &=  ~CSTOPB;
        tty.c_cflag     &=  ~CSIZE;
        tty.c_cflag     |=  CS8;

        tty.c_cflag     &=  ~CRTSCTS;           // no flow control
        tty.c_cc[VMIN]   =  1;                  // read doesn't block
        tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
        tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

        /* Make raw */
        cfmakeraw(&tty);

        /* Flush Port, then applies attributes */
        tcflush( serial_port_, TCIFLUSH );
        if ( tcsetattr ( serial_port_, TCSANOW, &tty ) != 0) {
           std::cout << "Error " << errno << " from tcsetattr" << std::endl;
           return false;
        }
        return true;
    }       
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "imu_reader");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string imu_name;
    pnh.getParam("sensor_name", imu_name);

    std::string port;
    float sampleFrequency; 
    nh.getParam(imu_name + std::string("/port"), port);
    nh.getParam(imu_name + std::string("/sample_frequency"), sampleFrequency);
    
   
    ROS_INFO("imu port %s ", port.c_str());
    auto reader = MPU9250Reader(port, sampleFrequency, imu_name, nh);

    while (ros::ok())
    {  
       reader.scan();
       ros::spinOnce();
    }


}
