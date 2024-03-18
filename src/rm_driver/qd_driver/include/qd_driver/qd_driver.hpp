#ifndef _QD_DRIVER_HPP_
#define _QD_DRIVER_HPP_
//C++相关
#include <memory> //内存操作库，封装了初始化内存，拷贝，移动，比较等方法
#include <vector>
#include <string>
#include <cstdio>
#include <math.h>
#include<iostream>
#include <termios.h> //linux串口通信编程库，描述终端输入输出并调用相关操作函数
//ROS相关
#include <ros/ros.h> //ROS标准库头文件
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h> //里程计消息库
#include <sensor_msgs/Imu.h> //imu消息库
#include <sensor_msgs/Range.h> //传感器数据截取
#include <geometry_msgs/Twist.h> //标准速度消息库
#include <geometry_msgs/TransformStamped.h> //TF坐标变换库
#include "tf/transform_datatypes.h"//转换函数头文件
#include <tf2/LinearMath/Quaternion.h> //TF四元数消息库
#include <tf2_ros/transform_broadcaster.h> //TF坐标变换广播器
#include <dynamic_reconfigure/server.h> //动态调参服务
#include "roborts_msgs/driver.h"//自定义消息
//boost库相关
#include <boost/asio.hpp> //异步输入输出核心库
#include <boost/asio/serial_port.hpp> //串口通信库
#include <boost/thread.hpp> //多线程库
#include <boost/system/system_error.hpp> //提供异常类来描述和识别错误
#include <boost/system/error_code.hpp> //跨平台的异常错误处理
#include <boost/bind.hpp> //绑定函数对象操作库

using namespace ros;
using namespace std;

/*
    通信协议：双帧头 + 帧长度 + 帧号(状态码) + 数据段 + 校验和
            0xAA 0x55 0x0B 0x01 (传输的数据段，高八位低八位，长度自由变化) 0xF5（实际是多少就是多少）   
            目前数据段包含Vx,Vy,Wz,六位数据位  （三轴里程计）
            （注释掉的是ax,ay,az,gx,gy,gz 六轴imu数据 不需要的话就自己删改）
        双帧头：抗数据干扰性更强
        帧长度：传输一帧数据的长度
        帧号：功能识别代号，状态机
        数据：高位在前，地位在后，长度可变，(8位，16位，数据自由组合)
        校验和： 前面数据累加和的低八位 */
/*
 * @brief 与下位机（电控）通信双帧头
*/
#define FRAME_HEADER_ONE 0xAA
#define FRAME_HEADER_TWO 0x55

/*
 * @brief 是否ROS_INFO测试信息
*/
#define DEBUG_RecvDataPacketHandle_HP           0
#define DEBUG_RecvDataPacketHandle_EVENT        0
#define DEBUG_RecvDataPacketHandle_CurrentMaxHP 0
#define DEBUG_RecvDataPacketHandle_POSE         0
#define DEBUG_RecvDataPacketHandle_BUFF         0
#define DEBUG_RecvDataPacketHandle_HURT         0
#define DEBUG_RecvDataPacketHandle_ALLOWANCE    0
#define DEBUG_RecvDataPacketHandle_RFID         0

#define DEBUG_CmdVelCallback_Vel_Data           0
#define DEBUG_RecCallback                       0
#define DEBUG_SERIAL                            0

/*
 * @brief 机器人数据处理周期，单位秒，20ms处理一次，因此odom话题设置为50HZ
*/
const float DATA_DURATION = 0.02f;

namespace qd_driver
{
/*
    * @brief 串口指针别名
*/
    typedef boost::shared_ptr<boost::asio::serial_port> serial_ptr;

/*
    * @brief 串口类别
    * @param rx_con_      接收计数器
    * @param rx_checksum_ 帧头部分校验和
    * @param rx_buf_[]    接收缓冲
*/
    uint8_t rx_con_ = 0;
    uint8_t rx_checksum_;
    uint8_t rx_buf_[60];

/*
 * @brief 底盘速度信息
 * @param linear_x  x方向线速度
 * @param linear_y  y方向线速度
 * @param angular_z z方向角速度
*/
    struct VelocityData
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };
    class SerialCommunication
    {
        private:
            
        public:
            SerialCommunication()
            {
                ROS_INFO("class SerialCommunication build~");
            }
            ~SerialCommunication()
            {
                ROS_INFO("class SerialCommunication delete~");
            }

    /*
        @brief 定义串口基本信息
        @param serial_port_baud_ 波特率
        @param serial_port_      串口号
        @param error_code_       错误码
        @param serial_ptr_       串口指针对象
        @param io_service_       io端口服务
        
    */
            int serial_port_baud_; 
            std::string serial_port_; 
    /*
        @brief 实例化串口基本信息
        @param error_code_       错误码
        @param serial_ptr_       串口指针对象
        @param io_service_       io端口服务
    */
            boost::shared_ptr<boost::asio::serial_port> serial_ptr_;
            boost::system::error_code error_code_;
            boost::asio::io_service  io_service_;
            
            bool OpenSerialPort(); //打开串口,配置串口参数，如波特率，数据位，停止位等
            void CloseSerialPort(); //关闭串口,删除对象,释放内存。
            
    };

    class Driver : public SerialCommunication
    {
        public:
            Driver();
            ~Driver();

        private:
            void RecCallback(); //多线程接收函数
            void RecvDataPacketHandle(uint8_t *buffer_data); //接收数据包,解析数据。
            void SendDataPacket(uint8_t *pbuf, uint8_t len); //发送数据包,变帧长通信协议。
            
            //实例化标准消息
            // nav_msgs::Odometry odom_msgs_; //里程计
            // sensor_msgs::Imu imu_msgs_; //imu

            //坐标系定义
            // std::string odom_frame_; 
            // std::string imu_frame_;
            std::string base_frame_; //哨兵车体

            //话题定义
            // std::string odom_topic_; //里程计话题
            // std::string imu_topic_; //imu话题
            std::string cmd_vel_topic_; //速度话题
            std::string referee_topic_;//裁判系统话题

            //发布器定义
            // ros::Publisher odom_pub_;
            // ros::Publisher imu_pub_;
            ros::Publisher referee_pub_;//裁判系统信息发布

            //订阅器定义
            ros::Subscriber cmd_vel_sub_; //非常重要,控制机器人移动的基底
            ros::Subscriber referee_sub_;//裁判系统信息订阅

            //实例化数据结构体
            struct VelocityData vel_data_;

            //odom,imu话题通信模型发布
            // void PublishOdom();
            // void PublishImu();
            //里程计：odom->base_frame的TF坐标变换
            // void PublishOdomTF();

            //速度消息回调函数
            void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    };
}

#endif