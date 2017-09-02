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
#include <dirent.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include <kvant/CryptString.h>
#include <kvant/Set_key.h>
#include <sensor_msgs/Image.h>

class Basic
{
    public:
        Basic();
        virtual ~Basic() {}

    protected:
        //virtual void getkey(std::string) = 0;
        //virtual void send(void) = 0;
        //virtual std::vector<uint8_t> recive(void) = 0;

        uint8_t T;   // длина ключа выделаненная под ответ от slave
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
        ros::ServiceServer getkey_srv;
        bool key_extend(kvant::Set_key::Request&, kvant::Set_key::Response&);

        //std::vector<uint8_t> key;
};

class Master: public Basic {
    public:
        Master();
        Master(std::string);
        void spin();

    protected:
        ros::NodeHandle nh_;
        ros::Subscriber sub_joy;
        ros::Publisher pub_open_chanel_data;
        ros::Subscriber sub_open_chanel_video;
        ros::Publisher pub_video;

        ros::ServiceClient set_key_client;

        bool camera_enable;
        struct dirent *ent;

        std::vector<char*> getAvalibleKeyNames();
        std::vector<uint8_t> readKey(char *key_name);
        bool expandKey();

        std::vector<uint8_t> make_data(std::vector<uint8_t> cmd, bool camera_enable);
        std::vector<uint8_t> encrypt(std::vector<uint8_t> data);
        std::vector<uint8_t> decrypt(std::vector<uint8_t> encrypt_video);
        void send(std::vector<uint8_t> data);
        void reciveCallback(const kvant::CryptString::ConstPtr& msg);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& msg_joy);

        // void encrypt_cb(const kvant::CryptStringConstPtr&);
        //std::string unencrypt(std::string);
        //bool getkey(std::string);

        //std::vector<uint8_t> key;
};

#endif /*KVANT_*/
