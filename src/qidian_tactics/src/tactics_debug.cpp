#include "ros/ros.h"
#include "dynamic_reconfigure/server.h"
#include "dynamic_reconfigure/client.h"
#include "qidian_tactics/driver.h"
#include "qidian_tactics/RobotParametersConfig.h"
#include "rosgraph_msgs/Clock.h"

ros::Publisher pub;
ros::Time start;
qidian_tactics::driver msgs;
bool is_pub = true;
class sim_time{
public:
    sim_time(ros::NodeHandle *nh)
    {
        this->nh_ = nh;
    }

    void get_sim_time()
    {
        ros::Subscriber sub = nh_->subscribe("/clock", 10, &sim_time::clockCallback,this);
        // 等待接收到仿真时间
        while (!sim_time_received) {
            ros::spinOnce();
            ros::Duration(0.1).sleep(); // 短暂等待
        }
    }

    void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
        sim_time_received = true;
        start = msg->clock;
    }
private:

bool sim_time_received = false;
ros::NodeHandle *nh_;
};


//处理动态参数
void get_dr_task(qidian_tactics::RobotParametersConfig &data,int32_t level)
{   
    ROS_INFO("%.1f", ros::Time::now().toSec() - start.toSec());
    //更新数据
        is_pub = data.is_pub;
        msgs.Central_area_status = data.Central_area_status;
        msgs.detect_enemy = data.detect_enemy;
        msgs.enemy_base_HP = data.enemy_base_HP;
        msgs.event_data = data.event_data;
        msgs.maximum_HP = data.maximum_HP;
        msgs.own_base_HP = data.own_base_HP;
        msgs.own_robot_HP = data.own_robot_HP;
        msgs.own_supply_area_Status = data.own_supply_area_Status;
        msgs.rfid_status =  data.rfid_status;
        msgs.shoot_num = data.shoot_num;
        msgs.Virtual_Shield_Status = data.Virtual_Shield_Status;
        

}


int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"Param_server_str");
    ros::NodeHandle nh;

    pub = nh.advertise<qidian_tactics::driver>("/referee_msgs",5);
    
    //动态参数服务
    dynamic_reconfigure::Server<qidian_tactics::RobotParametersConfig> Param_server;
    Param_server.setCallback(boost::bind(&get_dr_task,_1,_2));
    

    //初始化模拟时间
    sim_time(&nh).get_sim_time();
    msgs.start_time = start;

    while(ros::ok())
    {
        if (is_pub)
        {
            pub.publish(msgs);
        }
        msgs.header.stamp = ros::Time::now();
        ros::Duration(1).sleep();
        ros::spinOnce();    
    }
    return 0;
}
