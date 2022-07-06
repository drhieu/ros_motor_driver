#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <serial/serial.h>
#include <boost/algorithm/clamp.hpp>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string>

namespace SkippyRobot {

class RobotDriver {

    private:

    //  Sub
    ros::Subscriber cmdVel_subcriber_  // listen to cmd_vel linear and angular velocity

    // parameter variables
    serial::Serial ser;

    public:
    RobotDriver(ros::NodeHandle *nh);
    ~RobotDriver();
    void callbacktwist(const geometry_msgs::Twist& msg);
}; // class RobotDriver

} // namespace SkippyRobot