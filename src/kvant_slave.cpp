/*
 *  kvant_slave.cpp
 *
 *  Created on: 10.08.2017
 *          By: Nikoaly Dema
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include "kvant.h"
using namespace CryptoPP;
using namespace std;

Slave::Slave(std::string path): nh_("~"), Basic()
{
    video_sub = nh_.subscribe("/robotino/camera", 4, &Slave::robotino_video_cb, this);
    data_sub = nh_.subscribe("/open_channel_data", 4, &Slave::encrypt_data_cb, this);

    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    video_pub = nh_.advertise<geometry_msgs::Twist>("/open_chanel_video", 25, true);

    getkey_srv = nh_.advertiseService("set_key", &Slave::key_extend, this);
}

/* Action сервер, при получении запроса, дополняет имеющийся ключ с конца
 * значениями соответствующего поля запроса.
 */
bool Slave::key_extend(kvant::Set_key::Request& req, kvant::Set_key::Response& res)
{
    size_t j = key.size();
    for (size_t i = 0; i < req.key.size(); i++)
    {
        key.push_back(req.key[i]);
        std::cout << "Key is extended by: " << std::hex << key[j+i];
        //ROS_INFO(" %.2X", key[j+i]);
    }
    res.set = true;
    return true;
}

/* Прием шифротекста, дешифровка, парсинг и отправка управляющей команды для
 * Robotino, сохранение в очередь части ключа, доступной для использования
 * при отправки видео
 * Формат принимаемого сообщения следующий: |L|DATA|T|
 * L - длинна поля DATA в байтах
 * DATA - передаваемые данные, в данном случае - значения скоростей по трем
 *        компонентам x,y,z
 * T - кол-во байт ключа, доступных для отправки видео. Абсолютные положения
 *     доступных байт в ключе сохраняются в очередь для последующей отправки
 *     видео
 */
void Slave::encrypt_data_cb(const kvant::CryptStringConstPtr& msg)
{
    std::vector<uint8_t> data;
    geometry_msgs::Twist cmd_msg;

    size_t data_size = msg->crypt_string.size();
    if ((key.size() - pos_a) < data_size)
    {
        ROS_ERROR("[Slave]: Key size is less then recived data!");
        return;
    }

    // Дешифровка принимаемых данных
    for (size_t i = 0; i < data_size; i++) {
        data.push_back(msg->crypt_string[i]^key[pos_a+i]);
    }

    // Изменение текущей абсолютной позиции в ключе
    pos_a += (unsigned int)data_size;

    // Проверка наличия поля T
    if (data_size != (size_t)(data[0]+1))
    {
        // Если данные содержат поле T, то в очередь добавляются абсолютные
        // позиции байтов в ключе, доступные для передачи видео
        if ((pos_a+data[data_size-1] - pos_a + 1) != VIDEO_KEY_L) {
            ROS_ERROR("[Slave]: T field in the DATA is not equal to the "
                      "standard AES value!");
        }
        else
        {
            cam_key.push(std::make_pair(pos_a,pos_a+data[data_size-1]));
            pos_a += (unsigned int)data[data_size];
        }
    }

    // Если поле DATA не пустое, формируем и публикуем управляющее задание
    if (data[0] != 0)
    {
        cmd_msg.linear.x = (float)data[1];// * scale_linear_;
        cmd_msg.linear.y = (float)data[2];// * scale_linear_;
        cmd_msg.angular.z = (float)data[3];// * scale_angular_;
        cmd_vel_pub.publish(cmd_msg);
    }
    // ROS_INFO(" %zu -----", data_size);
    // ROS_INFO(" %.2X", msg->crypt_string[i]);
    // std::stringstream ss(data);
    //     ss >> vels[i];
}

void Slave::robotino_video_cb(const sensor_msgs::ImageConstPtr& msg)
{
    //
    kvant::CryptString video_msg;
    if (!cam_key.empty())
    {





        // std::pair<unsigned int, unsigned int> key_frame = cam_key.front();
        // cam_key.pop();
        //
        // byte key_aes[CryptoPP::AES::DEFAULT_KEYLENGTH];
        // byte iv_aes[CryptoPP::AES::BLOCKSIZE];
        //
        // std::vector<byte> plain, cipher;
        // HexEncoder encoder(new FileSink(std::cout));
        //
        // // memcpy(key_aes, &key[key_frame.first], VIDEO_KEY_L);
        // // memcpy(iv_aes,  &key[key_frame.first], VIDEO_KEY_L);
        // memset(key_aes, 0x00, sizeof(key_aes));
        // memset(iv_aes, 0x00, sizeof(iv_aes));
        //
        // std::string str("Attack at dawn!");
        // std::copy(str.begin(), str.end(), std::back_inserter(plain));
        // //std::copy(msg->data.begin(), msg->data.end(), std::back_inserter(plain));
        //
        // // std::cout << "[Slave] Plain text: ";
        // // encoder.Put(plain.data(), plain.size());
        // // encoder.MessageEnd();
        // // std::cout << std::endl;
        //
        // CBC_Mode<AES>::Encryption enc;
        // enc.SetKeyWithIV(key_aes, sizeof(key_aes), iv_aes, sizeof(iv_aes));
        //
        // // Make room for padding
        // cipher.resize(plain.size()+AES::BLOCKSIZE);
        // ArraySink cs(&cipher[0], cipher.size());
        //
        // ArraySource(plain.data(), plain.size(), true,
        //     new StreamTransformationFilter(enc, new Redirector(cs)));
        //
        // // Set cipher text length now that its known
        // cipher.resize(cs.TotalPutLength());

         std::cout << "Cipher text: ";
        // encoder.Put(cipher.data(), cipher.size());
        // encoder.MessageEnd();
         std::cout << std::endl;

        // CryptoPP::AES::Encryption en_aes(key_aes, CryptoPP::AES::DEFAULT_KEYLENGTH);
        // CryptoPP::CBC_Mode_ExternalCipher::Encryption en_cbc(en_aes, iv_aes);
        //
        // //unsigned char frame[HEIGHT*WIDTH*3];
        // //unsigned char chipt[HEIGHT*WIDTH*3];
        // std::vector<unsigned char> chipt;
        // chipt.resize(msg->data.size());
        // //for (size_t j = 0; j < msg->data.size(); j++) {
        // //    frame[j] = (unsigned char)msg->data[j];
        // //}
        //
        // CryptoPP::StreamTransformationFilter en_stf(en_cbc,
        //                     new CryptoPP::ArraySink(&chipt[0], chipt.size()));
        // en_stf.Put(reinterpret_cast<const unsigned char*>(&msg->data[0]),
        //            msg->data.size());
        // en_stf.MessageEnd();

        // for (size_t i = 0; i < cipher.size(); i++) {
        //     video_msg.crypt_string.push_back(cipher[i]);
        // }
        // video_pub.publish(video_msg);
    }
    else
    {
        ROS_ERROR("[Slave]: There are no key sequences permitting to transfer "
                  "video frames!");
        return;
    }

}

void Slave::spin()
{
    ros::Rate R(20);
    while(nh_.ok())
    {
        ros::spinOnce();
        //

        R.sleep();
    }
}
