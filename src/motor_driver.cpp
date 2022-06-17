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
    double leftmotor;
    double rightmotor;
    char velMsg[256];


    // linear_velocity = msg.linear.x;
    // angular_velocity = msg.angular.z;

    if((abs(msg.linear.x) <= 1) & (abs(msg.angular.z) <= 2))
    {
        rightmotor = msg.linear.x + (0.5 * msg.angular.z);
        leftmotor = msg.linear.x - (0.5 * msg.angular.z);

        rightmotor = boost::algorithm::clamp(round(rightmotor*127 + 127), 0, 254);
        leftmotor = boost::algorithm::clamp(round(leftmotor*127 + 127), 0, 254);

        int r_motor = rightmotor;
        int l_motor = leftmotor;

        // const uint8_t r_motor = rightmotor;
        // const uint8_t l_motor = leftmotor;
        // const uint8_t start_bit = 253;

        // const uint8_t checksum = (253 - (l_motor + r_motor)) % 255;

        // size_t st = sizeof(start_bit);
        // ser.write(&start_bit, st);

        // size_t lm = sizeof(l_motor);
        // ser.write(&l_motor, lm);

        // size_t rm = sizeof(r_motor);
        // ser.write(&r_motor, rm);

        // size_t ch = sizeof(checksum);
        // ser.write(&checksum, ch);

        sprintf(velMsg, "jx%dz%dy", r_motor, l_motor);
        ser.write(velMsg);

        // ROS_INFO("Linear velocity %3.2f    Angualar velocity %3.2f", linear_velocity, angular_velocity);
        ROS_INFO("Right motor %d    Left motor %d", r_motor, l_motor);
        // ROS_INFO("checksum %d",checksum);

        // ROS_INFO("[Listener] I heard: [%s]\n", msg->data.c_str());
    }
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

        //  if(ser.available()){
        //     ROS_INFO_STREAM("Reading from serial port");
        //     std::string result;
        //     result = ser.read(ser.available());
        //     ROS_INFO_STREAM("Read: " << result);
        // }   

    // Enter a loop, pumping callbacks
    //ros::spin();

        loop_rate.sleep();
    }
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////End main function//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////