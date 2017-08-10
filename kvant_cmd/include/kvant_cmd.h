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

#include <fstream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class kvant_Decoder
{
    public:
        kvant_Decoder(std::string);
        ~kvant_Decoder();

        void spin();

    protected:
        ros::NodeHandle nh_;
        ros::Publisher twist_pub;
        ros::Subscriber encrypt_sub;

        void encrypt_cb(const kvant_joy::CryptStringConstPtr&);
        std::string unencrypt(std::string);
        bool getkey(std::string);

        std::string key;
};

#endif /*KVANT_CMD*/
