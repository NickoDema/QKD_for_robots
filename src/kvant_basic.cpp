/*
 *  kvant_basic.cpp
 *
 *  Created on: 10.08.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include "kvant.h"

Basic::Basic() : T(16), pos_a(0), pos_r(0) {}

//
// void Basic::getkey() {}
// {
//     std::ifstream keyF(path, std::ios::binary | std::ios::ate);
//     size_t keyF_size = keyF.tellg();
//     keyF.seekg(0, std::ios::beg);
//     key.resize(keyF_size);
//     keyF.read( (char*)&key[0], keyF_size);
//     pos_counter = 0;
//
//     for (size_t i = 0; i < keyF_size; ++i)
//     {
//         std::string str;
//         str = std::to_string(key[i]);
//         ROS_INFO("crmsg %s %.2X", str.c_str(), key[i]);
//     }
//
//     return true;
// }
