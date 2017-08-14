#include <iostream>
#include <fstream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "kvant_joy/CryptString.h"

ros::Subscriber sub;
ros::Publisher pub;

char KEY_FILE_NAME[] = "/home/kirix/rosws/src/kvant/kvant_joy/key/00024b1f_Alice.key";
static int SHIFT = 0;
static int COUNT = 0;

/*
  Encrypting/Decrypting Function
  _data: array with data
  _fn: key file name
  return: vector with encrypted/decrypted data
*/
std::vector<unsigned char> XOR(unsigned char *_data, char *_fn)
{
  size_t data_size = 25;
  size_t fsize;
  //TODO: сделать проверку _data.size <= _fn.size
  //TODO: использовать файл ключа по кругу - для этого использовать static-переменную текущей позиции

  using namespace std;
  // reading key
  vector<unsigned char> key;
  ifstream _if(_fn, ios::binary);
  if (_if.is_open()) {
    _if.seekg(0, ios::end);
    fsize = _if.tellg();
    _if.seekg(0, ios::beg);
    key.resize(fsize);
    _if.read((char*) &key[0], fsize);
  } else {
    clog << "Opening key-file faild!" << endl;
  }

  // encrypting
  vector<unsigned char> ret(data_size);

  /*if data_size <= fsize*/
  // std::clog << SHIFT << std::endl  ;
  // int i = 0;
  // int pos = (SHIFT - 1) * data_size;
  // //    pos = 0; // !!!
  // while (i < data_size) {
  //   if(i + pos >= fsize) {
  //     SHIFT = 1;
  //     pos = (SHIFT - 1) * data_size;
  //   }
  //   ret[i] = _data[i] ^ key[i + pos];
  //   i++;
  // }

  for (int i = 0; i < data_size; i++) {
    if(SHIFT == fsize - 1) {
      SHIFT = 0;
    }
    ret[i] = _data[i] ^ key[i + SHIFT];

    std::clog << (int) ret[i] << " xor ";
    std::clog << (int) key[i + SHIFT] << " = ";
    std::clog << (int) _data[i] << std::endl;

   SHIFT++;
  }

  return ret;
}

/*
  Function for convert float to string
*/
std::string fts(float number) {
    std::ostringstream buff;
    buff << number;
    return buff.str();
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  std::clog << "Joy event!\n";
  if(pub.getNumSubscribers() > 0) {
    // geting values of axes pads
    float left_x = joy_msg->axes[0];  // left-right move
    float left_y = joy_msg->axes[1];  // forward--backward move
    float right_x = joy_msg->axes[2]; // rotation move
    float right_y = joy_msg->axes[3]; // dont useing

    // creating string with commands from gamepad
    // as "-0.23 0.01 -0.00"
    int size_data = 25;
    char data[size_data];
    sprintf(data, "%.2f %.2f %.2f", left_x, left_y, right_x);
    unsigned char *udata = reinterpret_cast<unsigned char*>(data);

    std::clog << "raw data: " << udata << std::endl;

    // make message for ros
    kvant_joy::CryptString crmsg;

    // encrypt
    std::clog << SHIFT << std::endl;
    std::vector<unsigned char> encrypt_data;
    encrypt_data = XOR(udata, KEY_FILE_NAME);

    std::clog << SHIFT << std::endl;

    // just test for -- decrypt
    // std::vector<unsigned char> decrypted_data;
    // decrypted_data = XOR(&encrypt_data[0], KEY_FILE_NAME);

    std::cout << "encr. data: "<< &encrypt_data[0] << std::endl;
    // std::cout << "decr. data: "<< &decrypted_data[0] << std::endl;

    // may be not need :)
    for(int i = 0; i < encrypt_data.size(); i++) {
      crmsg.crypt_string.push_back((uint8_t) encrypt_data[i]);
    }

    // for(int i = 0; i < 25; i++) {
    //   ROS_INFO("%d %.2X", (int) data[i], encrypt_data[i]);
    // }

    std::cout << "COUNT = " << COUNT << std::endl;
    crmsg.id = COUNT;
    COUNT++;
    pub.publish(crmsg);

    /* !!! if data_size <= fsize*/
    // SHIFT++;

    std::clog << "Encrypted msg sent!\n";
  } else {
    SHIFT = 0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_joy");
  ros::NodeHandle n;
  sub = n.subscribe("joy", 10, joyCallback);
  pub = n.advertise<kvant_joy::CryptString>("kvant_joy", 10);
  // ros::Duration(1).sleep();

  ros::Rate R(20);
  while(n.ok()) {
    ros::spinOnce();
    R.sleep();
  }

  // ros::spin();
  return 0;
}
