#include "kvant_joy.h"
#include "sensor_msgs/Joy.h"

#include "kvant_joy/CryptString.h"

#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
#include <sstream>

const std::string KEY_FILE_NAME = "00024b1f_Alice.key";
//
// std::vector<char> XOR(std::vector<unsigned char> _data, char * _fn)
// {
//   //TODO: сделать проверку _data.size <= _fn.size
//   //TODO: использовать файл ключа по кругу - для этого использовать static-переменную текущей позиции
//
//   using namespace std;
//   vector<unsigned char> key;
//   {
//     ifstream _if(_fn, ios::binary);
//     _if.seekg(0, ios::end);
//     size_t fsize = _if.tellg();
//     _if.seekg(0, ios::beg);
//     key.resize(fsize);
//
//     _if.read((char*)&key[0], fsize);
//   }
//
//   vector<char> ret(_data.size());
//   for (size_t i = 0; i < _data.size(); ++i)
//   {
//     // Мякотка
//     ret[i] = _data[i]^key[i];
//   }
//   return ret;
// }

std::string float_to_str(float number) {
    std::ostringstream buff;
    buff << number;
    return buff.str();
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // using namespace std;
  // cout << "left_x\t" << msg->axes[0] << endl;
  // cout << "left_y\t" << msg->axes[1] << endl;
  // cout << "right_x\t" << msg->axes[2] << endl;
  // cout << "right_y\t" << msg->axes[3] << endl;
  float left_x = msg->axes[0];
  float left_y = msg->axes[1];
  float rot = msg->axes[0];

  // "-0.12 -3.45 -6.78"
  char data[17];
	snprintf(data, sizeof(data), "%f %f %f", round(left_x * 100) / 100,
                                           round(left_y * 100) / 100,
                                           round(rot * 100) / 100);
	std::string data_str = data;
	std::cout << "data: " << data_str << std::endl;


  kvant_joy::CryptString crypt_msg;

  unsigned char udata[17];

  for(int i = 0; i < 17; i++) {
    crypt_msg.crypt_string = static_cast<unsigned char>(data[i]);
  }


  // crypt_msg.crypt_string = XOR(data, KEY_FILE_NAME);

  // std::cout << "crypted_data: " << crypt_msg.crypt_string;
  pub.publish(crypt_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_joy");
  ros::NodeHandle n;

  sub = n.subscribe("joy", 100, joyCallback);
  pub = n.advertise<kvant_joy::CryptString>("kvant_joy", 100);
  ros::spin();
  return 0;
}
