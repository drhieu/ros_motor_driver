/*
 * Author: Modou Dibba
 * Year: 2022
 *
 */

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////Program includes///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <serial/serial.h>
#include <boost/algorithm/clamp.hpp>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <string>

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////Global variables////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
serial::Serial ser;
const uint8_t left_motor = 49;
const uint8_t right_motor = 48;
const uint8_t positive = 48;
const uint8_t negative = 49;

long long int left_encoder = 0;
long long int right_encoder = 0;

//////////////////////////////////////////////////////////////////////////////////////////////
//////////Function impelmentations and declaration///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

// // useful functions
// void motor_direction_check()
// {
//         // Negative: 49   Positive: 48
//     switch(result[1])
//     {
//         case: 49
//                     ROS_INFO_STREAM("Neagtive direction");
//                     decrement_encoder_values();
//             break;
            
//                 case: 49
//                     ROS_INFO_STREAM("Positive direction");
//                     increment_encoder_values();
//             break;
//     }
// }

// void increment_encoder_values()
// {
//         // Left motor: 49    Right motor: 48
//     switch(result[0])
//     {
//         case: 49
//                 ROS_INFO_STREAM("Left motors");
                    
//                 std::string encoder_values = result.substr(2, msglength-3);
//                 left_encoder += stoi(encoder_values);
//                 // ROS_INFO_STREAM("Read: " << encoder_values);
//                 ROS_INFO("%lld",right_encoder); 
            
//                 break;
        
//         case: 48
//             ROS_INFO_STREAM("Right motors");
            
//             std::string encoder_values = result.substr(2, msglength-3);
//             right_encoder += stoi(encoder_values);
//             // ROS_INFO_STREAM("Read: " << encoder_values);
//             ROS_INFO("%lld",right_encoder); 
            
//             break;        		
//     }     	
// }

// void decrement_encoder_values()
// {
//         // Left motor: 49    Right motor: 48
//     switch(result[0])
//     {
//         case: 49
//                 ROS_INFO_STREAM("Left motors");
                    
//                 std::string encoder_values = result.substr(2, msglength-3);
//                 left_encoder -= stoi(encoder_values);
//                 // ROS_INFO_STREAM("Read: " << encoder_values);
//                 ROS_INFO("%lld",right_encoder); 
            
//                 break;
        
//         case: 48
//             ROS_INFO_STREAM("Right motors");
            
//             std::string encoder_values = result.substr(2, msglength-3);
//             right_encoder -= stoi(encoder_values);
//             // ROS_INFO_STREAM("Read: " << encoder_values);
//             ROS_INFO("%lld",right_encoder); 
            
//             break;        		
//     }        	
// }
        


// //Encoder values
// void encoders()
// {
//         // if(ser.available()){
//         //     std::string result;
//         //     result = ser.readline(ser.available());
//         //     int msglength = result.length();
//         //     //  ROS_INFO("%c",result[0]);

//         //     // Check motoor direction
//         //     if (result[0] == left_motor)
//         //     {    
//         //         ROS_INFO_STREAM("Left encoders");
//         //         // Check motor positon
//         //         if (result[1] == negative)
//         //         {
//         //             std::string encoder_values = result.substr(2, msglength-3);
//         //             left_encoder -= stoi(encoder_values);
//         //             // ROS_INFO_STREAM("Read: " << encoder_values);
//         //             ROS_INFO("%lld",left_encoder);
//         //         }
//         //         else if (result[1] == positive)
//         //         {
//         //             std::string encoder_values = result.substr(2, msglength-3);
//         //             left_encoder += stoi(encoder_values);
//         //             // ROS_INFO_STREAM("Read: " << encoder_values);
//         //             ROS_INFO("%lld",left_encoder);
//         //         }
//         //     }
//         //     else if (result[0] == right_motor)
//         //     {    
//         //         ROS_INFO_STREAM("Right encoders");
//         //         // Check motor positon
//         //         if (result[1] == negative)
//         //         {
//         //             std::string encoder_values = result.substr(2, msglength-3);
//         //             right_encoder -= stoi(encoder_values);
//         //             // ROS_INFO_STREAM("Read: " << encoder_values);
//         //             ROS_INFO("%lld",right_encoder);
//         //         }
//         //         else if (result[1] == positive)
//         //         {
//         //             std::string encoder_values = result.substr(2, msglength-3);
//         //             right_encoder += stoi(encoder_values);
//         //             // ROS_INFO_STREAM("Read: " << encoder_values);
//         //             ROS_INFO("%lld",right_encoder);
//         //         }
//         //     }
//         // }

//         motor_direction_check();
// }
/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////End function implementations and declarations/////////////////
////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////Function callback/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

// Topic messages callback
void twistCallback(const geometry_msgs::Twist& msg)
{
    double linear_velocity;
    double angular_velocity;
    char velMsg[256];


    linear_velocity = msg.linear.x;
    angular_velocity = msg.angular.z;

    linear_velocity = boost::algorithm::clamp(round(linear_velocity*127 + 127), 0, 255);
    angular_velocity = boost::algorithm::clamp(round(angular_velocity*127 + 127), 0, 255);

    int l_vel = linear_velocity;
    int a_vel = angular_velocity;

    sprintf(velMsg, "jx%dz%dy", l_vel, a_vel);
    ser.write("start");
    ser.write("start");
    ser.write(velMsg);

    // ROS_INFO("Linear velocity %3.2f    Angualar velocity %3.2f", linear_velocity, angular_velocity);
    ROS_INFO("Linear velocity %d    Angualar velocity %d", l_vel, a_vel);

    // ROS_INFO("[Listener] I heard: [%s]\n", msg->data.c_str());
}
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////End function callback/////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////




/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////Begin main function//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "motor_driver_node");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;


    // Subscribe to a given topic, in this case "chatter".
	//chatterCallback: is the name of the callback function that will be executed each time a message is received.
    ros::Subscriber cmdVel_sub = node.subscribe("/cmd_vel", 1000, twistCallback);


    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);
    while(ros::ok()){

        ros::spinOnce();
        // encoders();

    }

    // Enter a loop, pumping callbacks
    //ros::spin();

    loop_rate.sleep();

    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////End main function//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////