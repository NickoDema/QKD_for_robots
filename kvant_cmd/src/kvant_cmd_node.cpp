/*
 *  kvant_cmd_node.cpp
 *
 *  Created on: 10.08.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include "kvant_cmd.h"

kvant_Decoder::kvant_Decoder(std::string path): nh_("~")
{
    if (!getkey(path))
    {
        ROS_INFO("Can't open %s", path.c_str());
        exit(-1);
    }
    encrypt_sub = nh_.subscribe("/kvant_joy", 10, &kvant_Decoder::encrypt_cb, this);
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
}

bool kvant_Decoder::getkey(std::string path)
{
    std::ifstream keyF(path, std::ios::binary | std::ios::ate);
    size_t keyF_size = keyF.tellg();
    keyF.seekg(0, std::ios::beg);
    key.resize(keyF_size);
    keyF.read( (char*)&key[0], keyF_size);
    pos_counter = 0;

    for (size_t i = 0; i < keyF_size; ++i)
    {
        std::string str;
        str = std::to_string(key[i]);
        ROS_INFO("crmsg %s %.2X", str.c_str(), key[i]);
    }

    return true;
}

void kvant_Decoder::encrypt_cb(const kvant_joy::CryptStringConstPtr& msg)
{
    static int count = 0;
    ROS_INFO("id %d", msg->id);

    float vels[3];
    std::vector<unsigned char> data_v;
    geometry_msgs::Twist cmd_msg;

    size_t msg_size = msg->crypt_string.size();
    ROS_INFO(" %zu -----", msg_size);

    for (size_t i = 0; i < msg_size; ++i)
    {
        ROS_INFO(" %.2X", msg->crypt_string[i]);
    }
    ROS_INFO(" count = %d  before", pos_counter);
    for (size_t i = 0; i < msg_size; i++)
    {
        //ROS_INFO("i = %zu | count = %d", i, pos_counter);
        if (pos_counter == ((int)key.size()- 1)) {
            pos_counter = 0;
        }
        data_v.push_back((unsigned char)(msg->crypt_string[i]^key[i+pos_counter]));
        ROS_INFO(" %d xor %d = %d", (int)msg->crypt_string[i], (int)key[i+pos_counter],
                                    (int)data_v[i]);
        pos_counter++;
    }
    ROS_INFO(" count = %d  after", pos_counter);

    ROS_INFO("-------------------");
    for (size_t i = 0; i < data_v.size(); ++i)
    {
        std::string str;
        str = std::to_string(data_v[i]);
        ROS_INFO(" %d %.2X", (int)data_v[i], data_v[i]);
    }

    std::string data(data_v.begin(), data_v.end());
    ROS_INFO(" %s ", data.c_str());
    std::stringstream ss(data);
    for (int i=0; i<3; i++) {
        ss >> vels[i];
    }
    ROS_INFO("-------------------");
    cmd_msg.linear.x = vels[1];// * scale_linear_;
    cmd_msg.linear.y = vels[0];// * scale_linear_;
    cmd_msg.angular.z = vels[2];// * scale_angular_;

    cmd_vel_pub.publish(cmd_msg);
}

void kvant_Decoder::spin()
{
    ros::Rate R(20);
    while(nh_.ok())
    {
        ros::spinOnce();
        //

        R.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decoder");
    // if (argc >= 1) {
    //     kvant_Decoder vel_decoder(argv[1]);
    // }
    // else
    // {
    kvant_Decoder vel_decoder("/home/ram/programming/ROS/catkin_ws/src"
                              "/kvant/kvant_cmd/keys/00024b1f_Alice.key");
    //}
    vel_decoder.spin();

    return 0;
}
