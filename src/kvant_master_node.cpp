/*
 *  kvant_master_node.cpp
 *
 *  Created on: 10.08.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */
#include "kvant.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "master");
    Master ojct_name("/home/kirix/rosws/src"
                              "/kvant/keys/00024b1f_Alice.key");

    ojct_name.spin();
    return 0;
}
