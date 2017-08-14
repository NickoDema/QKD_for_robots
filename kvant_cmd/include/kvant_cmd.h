/*
 *  kvant_cmd.h
 *
 *  Created on: 10.08.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#ifndef KVANT_CMD_
#define KVANT_CMD_

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kvant_joy/CryptString.h>


class kvant_Decoder
{
    public:
        kvant_Decoder(std::string);
        //~kvant_Decoder();

        void spin();

    protected:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber encrypt_sub;

        void encrypt_cb(const kvant_joy::CryptStringConstPtr&);
        //std::string unencrypt(std::string);
        bool getkey(std::string);

        std::vector<uint8_t> key;
        int pos_counter;
};

#endif /*KVANT_CMD*/
