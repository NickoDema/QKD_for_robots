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
    //encrypt_sub = nh_.subscribe("/kvant_joy", 1, &kvant_Decoder::encrypt_cb, this);
}

bool kvant_Decoder::getkey(std::string path)
{
    std::ifstream keyF(path, std::ios::binary | std::ios::ate);
    size_t keyF_size = keyF.tellg();
    keyF.seekg(0, std::ios::beg);
    key.resize(keyF_size);
    keyF.read( (char*)&key[0], keyF_size);

    for (size_t i = 0; i < keyF_size; ++i)
    {
        std::string str;
        str = std::to_string(key[i]);
        ROS_INFO(" %s %.2X", str.c_str(), key[i]);
    }

    return true;
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
    //vel_decoder.spin();

    return 0;
}
