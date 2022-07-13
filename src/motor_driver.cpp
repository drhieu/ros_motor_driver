#include <math.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <stdio.h>

#include <boost/algorithm/clamp.hpp>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

class Motor_Driver {
   public:
    Motor_Driver();
   private:
    void incomingTwistCallback(const geometry_msgs::Twist::ConstPtr& vrtwist);  // For VR teleop
    void callbackBrake(const std_msgs::Bool::ConstPtr& updatebool);
    ros::NodeHandle nh_;
    std::mutex twist_update_mutex_;
    serial::Serial ser;
    std::string estop_trigger_topic_;
    bool hardbrakeonce = false;
    bool right_cw = true;
    bool left_cw = true;
    uint8_t brake = 0;
    double leftmotor;
    double rightmotor;

    int safetyflats_;
    int linear_, angular_;
    double l_scale_, a_scale_;
    std::atomic<bool> brake_state_;
    ros::Subscriber twist_sub_;
    std::thread motor_speed_update_thread_;
    void motors_control_loop(int time);
};

Motor_Driver::Motor_Driver() {
    twist_sub_ = nh_.subscribe<geometry_msgs::Twist>("/skippy/cmd_vel", 1, &Motor_Driver::incomingTwistCallback, this);
    brake_state_ = true;
    motor_speed_update_thread_ = std::thread([this]() { this->motors_control_loop(60); });
    try {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port ");
        throw -1;
    }

    if (ser.isOpen()) {
        ROS_INFO("Serial Port initialized");
    } else {
        throw -1;
    }
}

void Motor_Driver::motors_control_loop(int sleeptime) {
    while (true) {
        twist_update_mutex_.lock();
        uint8_t start_byte = 253;
        uint8_t mod_byte = 255;
        uint8_t r_motor = static_cast<uint8_t>(rightmotor);
        uint8_t l_motor = static_cast<uint8_t>(leftmotor);
        char velMsg[256];

        while (true) {
            uint8_t checksum = (((start_byte - (r_motor + l_motor)) % mod_byte) + mod_byte) % mod_byte;
            if (hardbrakeonce && brake == 1){
                if (right_cw)
                    r_motor = 253;
                else
                    r_motor = 0;
                if (left_cw)
                    l_motor = 253;
                else
                    l_motor = 0;
            }
            sprintf(velMsg, "%d@%d#%d&%d^%d\n", start_byte, l_motor, r_motor, brake, checksum);
            ser.write(velMsg);

            // ROS_INFO("Linear velocity %3.2f    Angualar velocity %3.2f", msg.linear.x, msg.angular.z);
            // ROS_INFO("Start byte: %d   Right motor:%d    Left motor:%d  Brakes:%d Checksum:%d", start_byte, r_motor, l_motor, brake, checksum);
        }

        twist_update_mutex_.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    }
}

void Motor_Driver::callbackBrake(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data == true)
        brake = 1;
    else
        brake = 0;
}

void Motor_Driver::incomingTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    twist_update_mutex_.lock();
    if ((abs(msg->linear.x) <= 1) && (abs(msg->angular.z) <= 2)) {
        rightmotor = msg->linear.x + (0.5 * msg->angular.z);
        leftmotor = msg->linear.x - (0.5 * msg->angular.z);

        rightmotor = boost::algorithm::clamp(round(rightmotor * 127 + 127), 0, 254);
        leftmotor = boost::algorithm::clamp(round(leftmotor * 127 + 127), 0, 254);
    }
    if (rightmotor > 127){
        right_cw = true;
    }else if (rightmotor < 127){
        right_cw = false;

    }
    if (leftmotor > 127){
        left_cw = true;
    }else if (leftmotor < 127){
        left_cw = false;

    }
    twist_update_mutex_.unlock();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Skippy Control Processor");
    Motor_Driver skippy;

    ros::spin();
}
