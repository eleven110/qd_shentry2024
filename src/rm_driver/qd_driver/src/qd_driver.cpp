#include "qd_driver.hpp"
#include "qd_referee_system.hpp"

/**
 * @brief 主函数，用于初始化 ROS 节点和创建 qd_driver::Driver 对象
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 返回执行状态，0 表示正常退出
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "qd_driver", ros::init_options::AnonymousName);
    ROS_INFO("qd_driver node init sucessfully!");
    qd_driver::Driver driver;
    return 0;
}

namespace qd_driver
{
    Driver::Driver()
    {   
        ROS_INFO("class Driver build~");
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_("~"); // 用于接收launch文件传入的参数

        /*
            private_nh_.param<>("a",b,"c");
            表示利用私有句柄调用参数实例化函数
            <>里面为参数模板类型，a传入的参数名，b为接收参数变量，c为默认参数名(不外部传参即使用这个)
        */

        // 串口通信、tf变换参数初始化
        private_nh_.param<std::string>("shaobing_port", this->serial_port_, "/dev/ttyUSB0");
        private_nh_.param<int>("shaobing_port_baud", this->serial_port_baud_, 115200);

        // 初始化速度话题名称信息
        private_nh_.param<std::string>("cmd_vel_topic", this->cmd_vel_topic_, "cmd_vel");
        private_nh_.param<std::string>("referee_topic", this->referee_topic_, "referee");

        // 初始化frame(坐标系)
        private_nh_.param<std::string>("base_frame", this->base_frame_, "base_footprint");

        // 实例化发布者对象
        //  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_,50);//消息队列长度为50，缓存区最多能保留50条旧信息，多出的会销毁
        //  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_,50);
        referee_pub_ = nh_.advertise<roborts_msgs::driver>(referee_topic_,1000);//裁判系统发布

        // 实例化订阅者对象
        /*
        subscribe订阅者的源码
            template<class M, class T>
            Subscriber subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(M), T* obj,
            const TransportHints& transport_hints = TransportHints()){
                ops.template initByFullCallbackType<M>(topic, queue_size, boost::bind(fp, obj, _1));
            }
            回调函数若是类成员函数，需要在函数名后面跟上this指针参数声明，以下是两种写法，bind是做函数绑定作用
        */

        cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(cmd_vel_topic_,100,boost::bind(&Driver::CmdVelCallback,this,_1));

        // 提示当前端口号和波特率
    #if DEBUG_SERIAL
        ROS_INFO("Shaobing Set serial %s at %d baud", serial_port_.c_str(), serial_port_baud_);
    #endif

        if (SerialCommunication::OpenSerialPort())
        {
            try
            {
                // 进入串口启动线程
            #if DEBUG_SERIAL
                ROS_INFO("thread--------------");
            #endif
                boost::thread serial_thread(boost::bind(&qd_driver::Driver::RecCallback, this));
            }
            catch (const std::exception &e)
            {

            #if DEBUG_SERIAL
                ROS_INFO("Open Serial_thread failed, Please check the serial port cable!");
            #endif
                throw e;
            }
        }

    /*
        ros::spin(); //将进程卡在此处，循环反复执行回调函数，不能执行后面语句，一般单独使用
        ros::spinOnce() 只会执行一次回调函数(没有循环)，可以接着执行后面语句,一般配合while使用
    */
        ros::spin();
    }

    Driver::~Driver()
    {
        static uint8_t vel_data[10];

        vel_data[0] = 0; // Vx
        vel_data[1] = 0;
        vel_data[2] = 0; // Vy
        vel_data[3] = 0;
        vel_data[4] = 0; // Wz
        vel_data[5] = 0;
        this->SendDataPacket(vel_data, 6);

        // 智能指针自动释放内存，不需要delete
        // ROS_INFO("Car is shutting down!"); // 提示信息
        SerialCommunication::CloseSerialPort();

        // ROS_INFO("class Driver delete~");

    }

    bool Driver::SerialCommunication::OpenSerialPort()
    {
    #if DEBUG_SERIAL
        ROS_INFO("openserial-----------------------------");
    #endif
        if (serial_ptr_) // 检测串口是否已经打开
        {
            ROS_INFO("The SerialPort is already opened!\n");
        }

        // 在堆区开辟一段内存打开串口
        serial_ptr_ = boost::shared_ptr<boost::asio::serial_port>(
            new boost::asio::serial_port(io_service_));
        ROS_INFO("new--------------");
        serial_ptr_->open(serial_port_, error_code_); // 打开当前设置的端口

        // 调用返回值error_code_判断串口是否正常打开
        if (error_code_)
        {

        #if DEBUG_SERIAL
            ROS_INFO("Open Port: %s Failed! ", serial_port_.c_str());
        #endif
            return false;
        }
        else
        {
            // 初始化串口参数,波特率，数据位，停止位，是否使用奇偶位，是否使用数据流量控制
            serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(serial_port_baud_));
            serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
            serial_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            serial_ptr_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

            return true;
        }
    }

    void Driver::SerialCommunication::CloseSerialPort()
    {
        // 关闭串口
        if (serial_ptr_)
        {
            serial_ptr_->cancel();
            serial_ptr_->close();
            serial_ptr_.reset();
        }

        io_service_.stop();
        io_service_.reset();
    }

/**
 * @brief 读取串口信息的回调函数
*/
    void Driver::RecCallback()
    {
    #if DEBUG_RecCallback
        ROS_INFO("RecCallback---------------");
    #endif
        uint8_t rec; // 接收数据变量
        static uint8_t vel_data[10];
        
        while (true)
        {
            // 通过智能指针维护数据读取，每次读取单个数据，提高效率
            boost::asio::read(*serial_ptr_.get(), boost::asio::buffer(&rec, 1), error_code_);
            //ROS_INFO("receieve:%x",rec);//打印接收的数据

            // 解析变帧长协议
            if (rx_con_ < 3) // 接收帧头 + 长度的数量
            {
                if (rx_con_ == 0) // 开始接收第一个帧头 0xAA
                {
                    if(rec == FRAME_HEADER_ONE)
                    {
                        rx_buf_[0] = rec; // 识别到第一个帧头便放入缓存数组内
                        rx_con_ = 1;      // 计数器指向下一位数据
                    }
                }
                else if (rx_con_ == 1) // 接收第二个帧头 0x55
                {

                    if (rec == FRAME_HEADER_TWO)
                    {
                        rx_buf_[1] = rec; // 识别到第二个帧头
                        rx_con_ = 2;      // 计数器指向下一位数据
                    }
                    else
                        rx_con_ = 0;
                }
                else
                {
                    // 接收数据包长度
                    rx_buf_[2] = rec; // 数据长度位     
                #if DEBUG_RecCallback
                    ROS_INFO("rx_buf_[2]:%d\r\n",rx_buf_[2]);                       
                #endif
                    rx_con_ = 3;      // 指向下一位数据
                    rx_checksum_ = 0;
                }
            }
            else // 此时rx_con_接收计数器=3，表示已完成帧头1，2，数据长度三位的检阅
            {
                if (rx_con_ < (rx_buf_[2] - 1)) // 数据并未到达末尾校位前，均是检阅数据段内data，数组下标从0开始
                {   
                    rx_buf_[rx_con_] = rec; // 以此开始存入数据段内的数据，直到遍历完成
                    rx_con_++;
                    rx_checksum_ += rec; // 累加前面所有数据，取该值低八位做校验和

                #if DEBUG_RecCallback
                    ROS_INFO("%x\r\n",rx_buf_[4]);
                    ROS_INFO("%x\r\n",rx_buf_[5]);
                #endif

                }
                else // 到达最后一位，其实最后一位是电控发来的数据校验和(八位)
                {
                    // 本次接收数据包完成，恢复默认值0等待接收下一次数据包
                    rx_con_ = 0;
                    if (rec == rx_checksum_) // 校验正确
                    {
                        // 对本次数据包进行处理
                        this->RecvDataPacketHandle(rx_buf_);
                    }
                }
            }
        }
    }

/**
 * @brief 处理接收到的数据的函数
 * (buffer_data[4] << 8) | buffer_data[5] 
    将buffer_data[4]左移8位后与buffer_data[5]进行或运算 结果获得16位
*/
    void Driver::RecvDataPacketHandle(uint8_t *buffer_data)
    {
        static uint16_t judge_cmd;
        static uint8_t i = 4;

        roborts_msgs::driver msgs;

        judge_cmd = (buffer_data[3] << 8) | buffer_data[4];
        if (judge_cmd == CMD_game_robot_HP_t)
        {
            game_robot_HP.red_7_robot_HP  = (buffer_data[i+1]   << 8) | buffer_data[i+2];
            game_robot_HP.red_outpost_HP  = (buffer_data[i+3]   << 8) | buffer_data[i+4];
            game_robot_HP.red_base_HP     = (buffer_data[i+5]   << 8) | buffer_data[i+6];
            game_robot_HP.blue_7_robot_HP = (buffer_data[i+7]   << 8) | buffer_data[i+8];
            game_robot_HP.blue_outpost_HP = (buffer_data[i+9]   << 8) | buffer_data[i+10];
            game_robot_HP.blue_base_HP    = (buffer_data[i+11]  << 8) | buffer_data[i+12];

            //发布血量信息
            // msgs.red_7_robot_HP  = game_robot_HP.red_7_robot_HP;
            // msgs.red_base_HP     = game_robot_HP.red_base_HP;
            // msgs.red_outpost_HP  = game_robot_HP.red_outpost_HP;
            // msgs.blue_7_robot_HP = game_robot_HP.blue_7_robot_HP;
            // msgs.blue_base_HP    = game_robot_HP.blue_base_HP;
            // msgs.blue_outpost_HP = game_robot_HP.blue_outpost_HP;
            uint16_t  red_hp_values[] = {game_robot_HP.red_7_robot_HP, game_robot_HP.red_base_HP, 
                                         game_robot_HP.red_outpost_HP};
            uint16_t blue_hp_values[] = {game_robot_HP.blue_7_robot_HP, game_robot_HP.blue_base_HP, 
                                         game_robot_HP.blue_outpost_HP};

            // for(int i = 0; i < 3; i++) {
            //     msgs.red_7_robot_HP = red_hp_values[0];
            //     msgs.red_base_HP    = red_hp_values[1];
            //     msgs.red_outpost_HP = red_hp_values[2];

            //     msgs.blue_7_robot_HP = blue_hp_values[0];
            //     msgs.blue_base_HP    = blue_hp_values[1];
            //     msgs.blue_outpost_HP = blue_hp_values[2];
            // }
            // msgs.event_data = false;
            // msgs.current_HP = false;
            // msgs.maximum_HP = false;
            // msgs.x = false;
            // msgs.y = false;
            // msgs.angle = false;
            // msgs.event_data = false;
            // msgs.event_data = false;

            // referee_pub_.publish(msgs);
            // ROS_INFO("%d",msgs.red_7_robot_HP);

        #if DEBUG_RecvDataPacketHandle_HP
            ROS_INFO("red_7_robot_HP:%04x\r\n,red_outpost_HP:%04x\r\n,red_base_HP:%04x\r\n,red_7_robot_HP:%d\r\n",
                     game_robot_HP.red_7_robot_HP, game_robot_HP.red_outpost_HP, game_robot_HP.red_base_HP);
            ROS_INFO("blue_7_robot_HP:%04x\r\n,blue_outpost_HP:%04x\r\n,blue_base_HP:%04x\r\n,blue_7_robot_HP:%04x\r\n",
                     game_robot_HP.blue_7_robot_HP, game_robot_HP.blue_outpost_HP, game_robot_HP.blue_base_HP);
        #endif
        }
        else if (judge_cmd == CMD_event_data_t)
        {
            event_data.event_data = (buffer_data[i+1] << 8) | buffer_data[i+2];

        #if DEBUG_RecvDataPacketHandle_EVENT
            ROS_INFO("event_data:%04x\r\n",event_data.event_data);
        #endif

        }
        else if (judge_cmd == CMD_ext_supply_projectile_action_t)
        {
            // ext_supply_projectile_action.current_HP = (buffer_data[i+1] << 8) | buffer_data[i+2];
            // ext_supply_projectile_action.maximum_HP = (buffer_data[i+3] << 8) | buffer_data[i+4];            

        #if DEBUG_RecvDataPacketHandle_CurrentMaxHP
            ROS_INFO("current_HP:%04x\r\n",ext_supply_projectile_action.current_HP);
            ROS_INFO("maximum_HP:%04x\r\n",ext_supply_projectile_action.maximum_HP);
        #endif

        }
        else if (judge_cmd == CMD_robot_pos_t)
        {
            robot_pos.x     = ((buffer_data[i+1]*1000) << 8) | (buffer_data[i+2]*1000);
            robot_pos.y     = ((buffer_data[i+3]*1000) << 8) | (buffer_data[i+4]*1000);
            robot_pos.angle = ((buffer_data[i+5]*1000) << 8) | (buffer_data[i+6]*1000);
    
        #if DEBUG_RecvDataPacketHandle_POSE
            ROS_INFO("robot_pos.x:%04x\r\n,robot_pos.y:%04x\r\n,robot_pos.angle:%04x\r\n",
                     robot_pos.x, robot_pos.y, robot_pos.angle);
        #endif

        }
        else if (judge_cmd == CMD_buff_t)
        {
            buff.defence_buff = buffer_data[i+1];
            buff.vulnerability_buff = buffer_data[i+2];
            buff.attack_buff = (buffer_data[i+3] << 8) | buffer_data[i+4];

        #if DEBUG_RecvDataPacketHandle_BUFF
            ROS_INFO("defence_buff:%04x\r\n,vulnerability_buff:%04x\r\n,attack_buff:%04x\r\n",
                     buff.defence_buff, buff.vulnerability_buff, buff.attack_buff);
        #endif

        }
        else if (judge_cmd == CMD_hurt_data_t)
        {
            hurt_data.armor_id = buffer_data[i+1];
            hurt_data.HP_deduction_reason = buffer_data[i+2];

        #if DEBUG_RecvDataPacketHandle_HURT
            ROS_INFO("armor_id:%04x\r\n,HP_deduction_reason:%04x\r\n",
                     hurt_data.armor_id, hurt_data.HP_deduction_reason);
        #endif

        }
        else if (judge_cmd == CMD_projectile_allowance_t)
        {
            projectile_allowance.projectile_allowance_17mm = (buffer_data[i+1] << 8) | buffer_data[i+2];

        #if DEBUG_RecvDataPacketHandle_ALLOWANCE
            ROS_INFO("projectile_allowance_17mm:%04x\r\n",
                     projectile_allowance.projectile_allowance_17mm);
        #endif

        }
        else if (judge_cmd == CMD_rfid_status_t)
        {
            rfid_status.rfid_status = (buffer_data[i+1] << 8) | buffer_data[i+2];

        #if DEBUG_RecvDataPacketHandle_RFID
            ROS_INFO("rfid_status:%08x\r\n",rfid_status.rfid_status);
        #endif

        }
    }

/**
    @brief 处理速度信息的回调函数
*/
    void Driver::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
    /*
        geometry_msgs/Vector3 linear
            float64 x
            float64 y
            float64 z
        geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z
    */
        static uint8_t vel_data[10];
    /*
        @param 数据转换，将十六位数据信息拆分为高八位+低八位进行存放
               速度 * 1000 是为了将float64位消息型小数变为int型整数
    */
        vel_data[0] = (int16_t)(msg->linear.x*1000) >>8;
        vel_data[1] = (int16_t)(msg->linear.x*1000) & 0xFF;
        vel_data[2] = (int16_t)(msg->linear.y*1000) >>8;
        vel_data[3] = (int16_t)(msg->linear.y*1000) & 0xFF;
        vel_data[4] = (int16_t)(msg->angular.z*1000)>>8;
        vel_data[5] = (int16_t)(msg->angular.z*1000)& 0xFF;

        #if DEBUG_CmdVelCallback_Vel_Data
        ROS_INFO("send 0:%d,1:%d,2:%d,3:%d,4:%d,5:%d",
                    vel_data[0],vel_data[1],vel_data[2],
                    vel_data[3],vel_data[4],vel_data[5]
                );        
        #endif

        // 通过串口发送数据
        this->SendDataPacket(vel_data, 6);
    }    

/**
    @brief 构建一个数据包并通过串口发送该数据包的函数
*/    
    void Driver::SendDataPacket(uint8_t *pbuf, uint8_t len)
    {
        uint8_t i, cnt;          // 发送计数器cnt
        uint8_t tx_checksum = 0; // 发送校验和
        uint8_t tx_buf[30];      // 发送缓冲

        if(len < 30) //判别是否超出长度
        {
            //取出数据
            tx_buf[0] = 0xAA; //帧头1
            tx_buf[1] = 0x55; //帧头2
            tx_buf[2] = len + 5; //根据输出的长度计算帧长度，len是数据位长度，5是双帧头+帧长度+帧识别码+校验和共5位
            tx_buf[3] = 0;

            for(i = 0; i < len; i++)
            {
                //帧第五位开始依次提取出，数据包位(一个数组)从下标0起始的数据
                tx_buf[4 + i] = *(pbuf + i);
            }
            //计算末尾位校验和
            cnt = 4 + len; //要累加多少位，这里除了末尾校验位是前四位+数据位
            for(i = 0; i < cnt; i++)
            {
                tx_checksum += tx_buf[i];
            }
            tx_buf[i] = tx_checksum; //赋值校验和结果到末位`
            // ROS_INFO("%x",tx_checksum);
            cnt = len+5;
            //发送数据
            boost::asio::write(*serial_ptr_.get(),boost::asio::buffer(tx_buf,cnt),error_code_);
        }
    }

}