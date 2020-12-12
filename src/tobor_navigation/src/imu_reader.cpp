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


class MPU9250Reader
{
public: 
    MPU9250Reader(std::string port_name){
        open_port(port_name);
    }
    ~MPU9250Reader(){}
    void scan()
    {
        read_port(read_buffer_);
        std::cout << "Response: " << read_buffer_ << std::endl;
    }

private:
    int serial_port_;
    char read_buffer_[1000];

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
    

    void open_port(std::string port_name)
    {
        /*https://stackoverflow.com/questions/18108932/reading-and-writing-to-serial-port-in-c-on-linux*/
        struct termios tty;
        memset (&tty, 0, sizeof tty);

        serial_port_ = open(port_name.c_str(), O_RDWR| O_NOCTTY );

        /* Error Handling */
        if ( tcgetattr ( serial_port_, &tty ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
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
        }
    }       
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "imu_reader");
    ros::NodeHandle nh;

    std::string port;
    nh.getParam("/forward_imu/port", port);
    std::cout << "Port " << port;
    auto reader = MPU9250Reader(port);

    while (ros::ok())
    {
       ros::spinOnce();
       reader.scan();
    }


}
