/*
 *  kvant_slave_node.cpp
 *
 *  Created on: 10.08.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include "kvant.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slave");
    Slave vel_decoder("/home/ram/programming/ROS/catkin_ws/src"
                              "/kvant/keys/00024b1f_Alice.key");

    vel_decoder.spin();
    return 0;
}
