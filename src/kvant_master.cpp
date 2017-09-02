/*
 *  kvant_master.cpp
 *
 *  Created on: 10.08.2017
 *       Email: kaartemov@gmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include "kvant.h"

Master::Master(std::string path) : Basic(), camera_enable(false) {
  ROS_INFO("Master init!");
  sub_joy = nh_.subscribe("joy", 10, &Master::joyCallback, this);
  sub_open_chanel_video = nh_.subscribe("open_chanel_video", 10, &Master::reciveCallback, this);
  pub_open_chanel_data = nh_.advertise<kvant::CryptString>("open_chanel_data", 10);
  pub_video = nh_.advertise<sensor_msgs::Image>("camera", 10);

  set_key_client = nh_.serviceClient<kvant::Set_key>("set_key");

  ros::Duration(1).sleep();
}

void Master::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  // ROS_INFO("Joy event!");
  if(pub_open_chanel_data.getNumSubscribers() > 0) {
    // geting values of axes pads
    float left_x = joy_msg->axes[0];  // left-right move
    float left_y = joy_msg->axes[1];  // forward--backward move
    float right_x = joy_msg->axes[2]; // rotation move
    // float right_y = joy_msg->axes[3]; // dont use

    // включение/отключение видео
    bool switch_camera = joy_msg->buttons[0]; // switch camera
    if (switch_camera == true) {
      camera_enable = !camera_enable;
      ros::Duration(0.5).sleep();
      if (camera_enable == true) {
        ROS_INFO("Camera turn on!");
      } else {
        ROS_INFO("Camera turn off!");
      }
    }

    // формируем команду управления только в том случае,
    // когда нажаты соответстующие кнопки
    std::vector<uint8_t> cmd;
    if (left_x != 0 || left_y != 0 || right_x != 0) {
      cmd.push_back(left_x);
      cmd.push_back(left_y);
      cmd.push_back(right_x);
    }

    std::vector<uint8_t> data;
    data = make_data(cmd, camera_enable);
    if (data.size() > 0) {
      std::clog << "Sent data: ";
      send(data);
      for (int i = 0; i < data.size(); i++) {
        std::clog << (int) data[i] << " ";
      }
      std::clog << std::endl;
    }
  }
}

/*
  В зависимости от сценария работа (определяется геймпадом) формирует data
  Возможны четыре сценария:
  L|cmd|T
  L|cmd
  L|T
  - --- возвращается пустой вектор
*/
std::vector<uint8_t> Master::make_data(std::vector<uint8_t> cmd,
                                       bool camera_enable) {
  std::vector<uint8_t> data;
  int cmd_len = cmd.size();
  // если есть команда управления
  if (cmd_len != 0) {
    data.push_back(cmd_len);
    for(int i = 0; i < cmd_len; i++) {
      data.push_back(cmd[i]);
    }
  }
  if (camera_enable == true) {
    data.push_back(0);  // L == 0; т.е. команды управления нет (TODO: ИЗБАВИТЬСЯ)
    data.push_back(T);
  }
  return data;
}

std::vector<uint8_t> Master::readKey(char *key_name) {
  size_t fsize;
  // reading key
  std::vector<uint8_t> key;
  std::ifstream _if(key_name, std::ios::binary);
  if (_if.is_open()) {
    _if.seekg(0, std::ios::end);
    fsize = _if.tellg();
    _if.seekg(0, std::ios::beg);
    key.resize(fsize);
    _if.read((char*) &key[0], fsize);
  } else {
    std::clog << "Opening key-file faild!" << std::endl;
  }
  return key;
}

/*
  Выписывает в вектор названия всех файлов из папки
  ../keys/
*/
std::vector<char*> Master::getAvalibleKeyNames() {
  std::vector<char*> key_names;
  DIR *dir;
  int next = 0;
  if ((dir = opendir("../keys/")) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      // костыль, чтобы не тянуть . и ..
      if (next < 2) {
        next++;
        continue;
      }
      char* file_name = ent->d_name;
      key_names.push_back(file_name);
    }
  } else {
    ROS_INFO("Folder 'keys' don't exist!");
  }
  return key_names;
}

/*
  TODO: удаление/перемещение отработанных ключей
  Расширяет ключ в переменной key.
  1. Выписывает именя ключей из папки с ключами в вектор
  Если ключи там есть, то:
  2. Считывает последний ключ (очередность формируется в getAvalibleKeyNames(.))
  3. создается переменная для нового ключа на робота
  4. К текущему ключу key в конце добавляется новосчитанный
  5. Этот же считанный ключ отправляется на робота
  Иначе:
  TODO: ожидание появления новых ключей в папке
*/
bool Master::expandKey() {
  std::vector<char*> key_names = getAvalibleKeyNames();
  if (key_names.size() > 0) {
    std::vector<uint8_t> new_key = readKey(key_names[key_names.size() - 1]);
    kvant::Set_key srv;
    for(int i = 0; i < new_key.size(); i++) {
      key.push_back(new_key[i]);
      srv.request.key.push_back(new_key[i]);
    }
    set_key_client.call(srv);
  } else {
    // TODO: wait :) ????
  }
  return 1;
}

void Master::send(std::vector<uint8_t> data) {
  std::vector<uint8_t> encrypt_data = Master::encrypt(data);
  kvant::CryptString msg;
  for(int i = 0; i < encrypt_data.size(); i++) {
    msg.crypt_string.push_back(encrypt_data[i]);
  }
  pub_open_chanel_data.publish(msg);
}

std::vector<uint8_t> Master::encrypt(std::vector<uint8_t> data) {
  std::vector<uint8_t> encrypt_data;

  int need_memory = data.size() + T;
  if (need_memory < key.size() - pos_a) {
    expandKey();
  }
  for (int i = 0; i < data.size(); i++) {
    encrypt_data.push_back(data[i] ^ key[i + pos_a]);
  }
  return encrypt_data;
}

std::vector<uint8_t> Master::decrypt(std::vector<uint8_t> encrypt_video) {
  std::vector<uint8_t> data;
  for (int i = 0; i < encrypt_video.size(); i++) {
    data[i] = encrypt_video[i] ^ key[i + pos_a];
  }
  return data;
}

void Master::reciveCallback(const kvant::CryptString::ConstPtr& msg) {
  std::vector<uint8_t> en_data_msg = msg->crypt_string;
  sensor_msgs::Image image;
  image.data = Master::decrypt(en_data_msg);
  Master::pub_video.publish(image);
}

void Master::spin() {
  ros::Rate R(20);
  while(nh_.ok()) {
    ros::spinOnce();
    R.sleep();
  }
}
