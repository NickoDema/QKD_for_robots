#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "std_msgs/String.h"
#include <iostream>
#include <sstream>

std::string float_to_str(float number){
    std::ostringstream buff;
    buff<<number;
    return buff.str();
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  // "0.11 0.22 0.33"

  using namespace std;
  cout << "left_x\t" << msg->axes[0] << endl;
  cout << "left_y\t" << msg->axes[1] << endl;
  cout << "right_x\t" << msg->axes[2] << endl;
  cout << "right_y\t" << msg->axes[3] << endl;
  float left_x = msg->axes[0];
  float left_y = msg->axes[1];
  float rot = msg->axes[0];

  char buff[100];
	snprintf(buff, sizeof(buff), "%d %d %d", left_x, left_y, rot);
	std::string buffAsStdStr = buff;

	std::cout << buffAsStdStr;

  std::string data = std::str()

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_joy");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joy", 100, joyCallback);
  ros::spin();
  return 0;
}
