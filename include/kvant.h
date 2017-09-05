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

#define HEIGHT 480              // Высота изображения в пикселях
#define WIDTH 640               // Ширина изображения в пикселях
#define VIDEO_KEY_L 16          // Длинна ключа для одного фрейма в байтах
//#define ARRAY_SIZE(array) (sizeof((array))/sizeof((array[0])))

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <vector>
#include <dirent.h>
#include <queue>
#include <string>
using namespace std;

#include "crypto++/cryptlib.h"
#include <crypto++/aes.h>
#include <crypto++/modes.h>
#include <crypto++/filters.h>
#include "crypto++/hex.h"
#include "crypto++/files.h"
using namespace CryptoPP;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include <kvant/CryptString.h>
#include <kvant/Set_key.h>
#include <sensor_msgs/Image.h>
#include <kvant/Aes.h>


class Basic
{
    public:
        Basic();
        virtual ~Basic() {}

    protected:
        ros::NodeHandle nh_;
        uint8_t T;
        unsigned int pos_a;             // absolute position in the key
        unsigned int pos_r;             // relative position in the key
        int pos_counter;

        std::vector<uint8_t> key;
        std::queue<std::pair<unsigned int, unsigned int> > cam_key;
        ros::ServiceClient aes_client;

};

class Slave: public Basic
{
    public:
        Slave(std::string);

        void spin();

    protected:
        ros::Subscriber data_sub;
        ros::Publisher cmd_vel_pub;
        ros::Subscriber video_sub;
        ros::Publisher video_pub;

        ros::ServiceServer getkey_srv;
        bool key_extend(kvant::Set_key::Request&, kvant::Set_key::Response&);
        void encrypt_data_cb(const kvant::CryptStringConstPtr&);
        void robotino_video_cb(const sensor_msgs::ImageConstPtr&);

        // bool key_extend(kvant::Set_key::Request&, kvant::Set_key::Response&);

        //std::vector<uint8_t> key;
};

class Master: public Basic {
    public:
        Master();
        Master(std::string);
        void spin();

    protected:
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

// class Master: public Basic
// {
//     public:
//         Master(std::string) : camera_enable(false) {};
//         //~Master();
//
//         void spin();
//
//     protected:
//         ros::NodeHandle nh_;
//         ros::Publisher pub_open_chanel_data;
//         ros::Subscriber sub_joy;
//         bool camera_enable;
//
//         std::vector<uint8_t> make_data(std::vector<uint8_t> cmd, uint8_t T);
//         std::vector<uint8_t> encrypt(std::vector<uint> data);
//         std::vector<uint8_t> decrypt(std::vector<uint> encrypt_video) {
//         void send(std::vector<uint8_t> data);
//         void reciveCallback(const kvant::CryptString::ConstPtr& msg);
//         void joyCallback(const sensor_msgs::Joy::ConstPtr& msg_joy)
//
//         // void encrypt_cb(const kvant::CryptStringConstPtr&);
//         //std::string unencrypt(std::string);
//         //bool getkey(std::string);
//
//         //std::vector<uint8_t> key;
// };

#endif /*KVANT_*/
