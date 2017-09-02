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
        break;
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
        cam_key.push(std::make_pair(pos_a,pos_a+data[data_size-1]);
        pos_a += (unsigned int)data[data_size];
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

void robotino_video_cb(const sensor_msgs::ImageConstPtr& msg)
{
    //
    kvant::CryptString video_msg;

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
