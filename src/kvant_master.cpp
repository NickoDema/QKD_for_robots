/*
 *  kvant_master.cpp
 *
 *  Created on: 10.08.2017
 *       Email: kaartemov@gmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */
#include "kvant.h"

/*
  В зависимости от сценария работа (определяется геймпадом) формирует data
  Возможны четыре сценария:
  L|cmd|T
  L|cmd
  L|T
  - --- возвращается пустой вектор
*/
std::vector<uint8_t> Master::make_data(std::vector<uint8_t> cmd, uint8_t T) {
  std::vector<uint8_t> data;
  cmd_len = cmd.length();
  // если есть команда управления
  if (cmd_len != 0) {
    data.push_back(cmd_len);
    for(int i = 0; i < cmd_len; i++) {
      data.push_back(cmd_len[i])
    }
  } else {
    data.push_back(0);  // L == 0; команды управления нет
  }
  //
  if (T != 0) {
    data.push_back()
  }
  return data;
}

std::vector<uint8_t> Master::encrypt(std::vector<uint> data) {
  std::vector<uint> encrypt_data;
  for (int i = 0; i < data.length(); i++) {
    encrypt_data[i] = data[i] ^ key[i + pos_a];
  }
  return encrypt_data;
}

std::vector<uint8_t> Master::decrypt(std::vector<uint> encrypt_video) {
  std::vector<uint> data;
  for (int i = 0; i < encrypt_video.length(); i++) {
    data[i] = encrypt_video[i] ^ key[i + pos_a];
  }
  return data;
}

void Master::send(std::vector<uint8_t> data) {
  std::vector<uint8_t> encrypt_data = encrypt(data);
  pub_open_chanel_data.publish(encrypt_data);
}

void Master::reciveCallback(const kvant::CryptString::ConstPtr& msg) {
  Image image = decrypt(msg);
  pub_video.publish(image);
}

void Master::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  std::clog << "Joy event!\n";
  if(pub_open_chanel_data.getNumSubscribers() > 0) {
    // geting values of axes pads
    float left_x = joy_msg->axes[0];  // left-right move
    float left_y = joy_msg->axes[1];  // forward--backward move
    float right_x = joy_msg->axes[2]; // rotation move
    // float right_y = joy_msg->axes[3]; // dont use

    // включение/отключение видео
    bool switch_camera = joy_msg->buttons[0]; // switch camera
    if (switch_camera == true) {
      switch_camera = !switch_camera;
    }

    std::vector<uint8_t> cmd;
    cmd.push_back(left_x);
    cmd.push_back(left_y);
    cmd.push_back(right_x);

    std::vector<uint8_t> data;
    if (switch_camera == true) {
      data = make_data(cmd, T);
    } else {
      data = make_data(cmd, 0);
    }
    send(data);
}

void Master::spin() {
  ros::init(argc, argv, "master");
  sub_joy = nh_.subscribe("joy", 10, joyCallback);
  pub_open_chanel_data = nh_.advertise<kvant::CryptString>("open_chanel_data", 10);
  sub_open_chanel_video = nh_.subscribe("open_chanel_video", 10, reciveCallback);
  pub_video = nh_.advertise<sensor_msgs::Image>("camera", 10);
  // ros::Duration(1).sleep();

  ros::Rate R(20);
  while(n.ok()) {
    ros::spinOnce();
    R.sleep();
  }

}
