/*
 *  kvant.h
 *
 *  Created on: 10.08.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#ifndef KVANT_
#define KVANT_

// #include <filesystem>       //working with dirs
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include <kvant/CryptString.h>


class Basic
{
    public:

        virtual ~Basic() {}

    protected:
        virtual void getkey(std::string) = 0;
        //virtual void send(void) = 0;
        //virtual std::vector<uint8_t> recive(void) = 0;

        unsigned int pos_a;             //absolute position in the key
        unsigned int pos_r;             //relative position in the key
        int pos_counter;

        std::vector<uint8_t> key;

};

class Slave: public Basic
{
    public:
        Slave(std::string);
        //~Slave();

        void spin();

    protected:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber encrypt_sub;

        void encrypt_cb(const kvant::CryptStringConstPtr&);
        //std::string unencrypt(std::string);
        bool getkey(std::string);

        //std::vector<uint8_t> key;
};

class Master: public Basic
{
    public:
        Master(std::string);
        //~Master();

        void spin();

    protected:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber encrypt_sub;

        void encrypt_cb(const kvant::CryptStringConstPtr&);
        //std::string unencrypt(std::string);
        //bool getkey(std::string);

        //std::vector<uint8_t> key;
};

#endif /*KVANT_*/
