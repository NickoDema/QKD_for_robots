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
  sub_joy = nh_.subscribe("joy", 10, &Master::joyCallback, this);
  sub_open_chanel_video = nh_.subscribe("open_chanel_video", 10, &Master::reciveCallback, this);
  pub_open_chanel_data = nh_.advertise<kvant::CryptString>("open_chanel_data", 10);
  pub_video = nh_.advertise<sensor_msgs::Image>("camera", 10);

  set_key_client = nh_.serviceClient<kvant::Set_key>("set_key");
  ros::Duration(1).sleep();
  // expandKey();
  ROS_INFO("Master init!");
}

void Master::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
  // ROS_INFO("Joy event!");
  if(pub_open_chanel_data.getNumSubscribers() > 0) {
    // geting values of axes pads
    float left_x = joy_msg->axes[0];  // left-right move
    float left_y = joy_msg->axes[1];  // forward--backward move
    float right_x = joy_msg->axes[2]; // rotation move
    // float right_y = joy_msg->axes[3]; // dont use

    // команда остановки
    bool stop_robot = joy_msg->buttons[3];

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
      cmd.push_back(left_x * 125 + 125);
      cmd.push_back(left_y * 125 + 125);
      cmd.push_back(right_x * 125 + 125);
    } else {
      if (stop_robot == true) {
        cmd.push_back(125);
        cmd.push_back(125);
        cmd.push_back(125);
      }
    }

    std::vector<uint8_t> data;
    data = make_data(cmd, camera_enable);
    if (data.size() > 0) {

      std::clog << "---\nData sent: ";
      for (int i = 0; i < data.size(); i++) {
        std::clog << static_cast<int>(data[i]) << " ";
      }
      std::clog << std::endl;

      send(data);
    }
  }
  ros::Duration(0.1).sleep();
}

void Master::send(std::vector<uint8_t> data) {
  std::vector<uint8_t> encrypt_data = Master::encrypt(data);

  std::clog << "Data encrypted: ";
  for (int i = 0; i < encrypt_data.size(); i++) {
    std::clog << static_cast<int>(encrypt_data[i]) << " ";
  }

  // decryption test for only
  // std::clog << "\n";
  // pos_a -= data.size() + T;
  // std::vector<uint8_t> decrypt_data = Master::encrypt(encrypt_data);
  //
  // std::clog << "Data decrypted: ";
  // for (int i = 0; i < decrypt_data.size(); i++) {
  //   std::clog << static_cast<int>(decrypt_data[i]) << " ";
  // }

  std::clog << "\n---";

  kvant::CryptString msg;
  for(int i = 0; i < encrypt_data.size(); i++) {
    msg.crypt_string.push_back(encrypt_data[i]);
  }
  pub_open_chanel_data.publish(msg);
}

std::vector<uint8_t> Master::encrypt(std::vector<uint8_t> data) {
  std::vector<uint8_t> encrypt_data;

  int need_memory = data.size() + T;
  if (need_memory > key.size() - pos_a) {
    ROS_INFO("Expanding key...");
    expandKey();
  }

  std::clog << "Need length of the key: " << need_memory
            << "\nThe rest of the key: " << key.size() - pos_a
            << "\nPosition on the key (abs.): "<< pos_a
            << "\n";

  for (int i = 0; i < data.size(); i++) {
    encrypt_data.push_back(data[i] ^ key[i + pos_a]);
    // std::clog << static_cast<int>(data[i]) << " ^ "
    //           << static_cast<int>(key[i + pos_a]) << " = "
    //           << static_cast<int>(data[i] ^ key[i + pos_a]) << std::endl;
  }
  pos_a += data.size() + T;
  return encrypt_data;
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
  ROS_INFO("Current key size: %d", static_cast<int>(key.size()));
  std::vector<char*> key_names = getAvalibleKeyNames();
  if (key_names.size() > 0) {
    std::vector<uint8_t> new_key = readKey(key_names[key_names.size() - 1]);
    ROS_INFO("Key was expanded by %s", key_names[key_names.size() - 1]);
    // TODO: move/remove key file!
    kvant::Set_key srv;
    for(int i = 0; i < new_key.size(); i++) {
      key.push_back(new_key[i]);
      srv.request.key.push_back(new_key[i]);
    }
    set_key_client.call(srv);
  } else {
    std::clog << "Waiting for kyes...";
    // TODO: wait :) ????
  }
  ROS_INFO("Fresh key size: %d", static_cast<int>(key.size() - pos_a));
  return 1;
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
    if (data.size() == 0) {
      data.push_back(0);  // L == 0; т.е. команды управления нет (TODO: ИЗБАВИТЬСЯ)
    }
    data.push_back(T);
    cam_key.push(std::make_pair(pos_a, pos_a + T));
  }
  return data;
}

std::vector<uint8_t> Master::readKey(char *key_name) {
  std::clog << key_name << std::endl;
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
  char ex_key[] = ".key";
  std::vector<char*> key_names;
  DIR *dir;
  if ((dir = opendir("keys/")) != NULL) {
    ROS_INFO("Finding keys in folder 'kvant/key/'...");
    // ROS_INFO("Avalible keys in directory: ");
    while ((ent = readdir(dir)) != NULL) {
      char* file_name = ent->d_name;
      int length_file_name = strlen(file_name);
      // Рассматриваем файлы длиной строго больше чем 4 символа.
      if (length_file_name >= 5) {
        bool isKey = true;
        for(int i = 0; i < 4; i++) {
          // Выбираем файлы только с расширением .key
          if (ex_key[i] != file_name[length_file_name - 1 - (3 - i)]) {
            isKey = false;
          }
        }
        if (isKey == true) {
          std::string path = "keys/";
          std::string fname(file_name);
          std::string full = path + fname;
          char *full_name = new char[full.length() + 1];
          strcpy(full_name, full.c_str());
          key_names.push_back(full_name);
        }
      }
    }
  } else {
    ROS_INFO("Folder 'keys' don't exist!");
  }
  return key_names;
}

std::vector<uint8_t> Master::decrypt(std::vector<uint8_t> encrypt_video) {

  std::vector<uint8_t> data;

  std::pair<unsigned int, unsigned int> key_frame = cam_key.front();
  cam_key.pop();

  byte key_aes[CryptoPP::AES::DEFAULT_KEYLENGTH];
  byte iv_aes[CryptoPP::AES::BLOCKSIZE];

  std::vector<byte> recover, cipher;
  HexEncoder encoder(new FileSink(std::cout));

  memcpy(key_aes, &key[key_frame.first], VIDEO_KEY_L);
  memcpy(iv_aes,  &key[key_frame.first], VIDEO_KEY_L);

  std::copy(encrypt_video.begin(), encrypt_video.end(),
            std::back_inserter(cipher));

  CBC_Mode<AES>::Decryption dec;
  dec.SetKeyWithIV(key_aes, sizeof(key_aes), iv_aes, sizeof(iv_aes));

  recover.resize(cipher.size());
  ArraySink rs(&recover[0], recover.size());

  ArraySource(cipher.data(), cipher.size(), true,
    new StreamTransformationFilter(dec, new Redirector(rs)));

  recover.resize(rs.TotalPutLength());

  std::copy(recover.begin(), recover.end(), std::back_inserter(data));

  return data;

  // std::vector<uint8_t> video;
  // if (!cam_key.empty()) {
  //   std::pair<unsigned int, unsigned int> key_frame = cam_key.front();
  //   cam_key.pop();
  //   kvant::Aes aes_srv;
  //   // 1. select mode of aes encr/decr
  //   aes_srv.request.mode = 0; // decryption
  //   // 2. add to service request KEY
  //   for (int i = key_frame.first; i < key_frame.second; i++) {
  //     aes_srv.request.key.push_back(key[i]);
  //   }
  //   // 3. add to service request DATA
  //   aes_srv.request.req_data = video;
  //   aes_client.call(aes_srv);
  //   video = aes_srv.response.resp_data;
  // }
  // return video;
}

void Master::reciveCallback(const kvant::CryptString::ConstPtr& msg) {
  std::vector<uint8_t> en_data = msg->crypt_string;
  sensor_msgs::Image image;
  image.header.stamp = ros::Time();
  image.header.frame_id = "/open_channel_data";
  image.height = HEIGHT;
  image.width = WIDTH;
  image.encoding = "rgb8";
  image.step = WIDTH*3;
  image.data = Master::decrypt(en_data);
  Master::pub_video.publish(image);
}

void Master::spin() {
  ros::Rate R(20);
  while(nh_.ok()) {
    ros::spinOnce();
    R.sleep();
  }
}
