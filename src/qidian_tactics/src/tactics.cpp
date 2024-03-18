#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/Int8.h"
#include "qidian_tactics/driver.h"

class goal_point
{
public:
    goal_point()
    {
        Invalid_target_point = true;
    }
    goal_point(const float &x,const float &y)
    {
        //position
        this->x = x;
        this->y = y;
        //orientation
        this->Z = 0;
        this->W = 1;
        Invalid_target_point = false;
    }
    goal_point(const float &x,const float &y,const float &Z,const float &W)
    {
        //position
        this->x = x;
        this->y = y;
        //orientation
        this->Z = Z;
        this->W = W;
        Invalid_target_point = false;
    }
    float x;
    float y;
    float Z;
    float W;
    //用于定义是否为有效坐标点，无效坐标点用于取消导航任务，有效坐标点用于导航，无参构造函数默认为无效坐标点
    bool Invalid_target_point;
};

class Navigator{

public:    
    Navigator(){}
    Navigator(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *client,ros::NodeHandle *nh_)
    {
        this->client=client;
        this->nh_=nh_;
    }
    void Setptr(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *client,ros::NodeHandle *nh_)
    {
        this->client=client;
        this->nh_=nh_;
    }

    //调试查看导航状态
    void debug_clientstate()
    {
            if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE)
            {
                ROS_INFO("ACTIVE");
            }
            if (client->getState() == actionlib::SimpleClientGoalState::ABORTED)
            {
                ROS_INFO("ABORTED");
            }
            if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("SUCCEEDED");
            }
            if (client->getState() == actionlib::SimpleClientGoalState::PENDING)
            {
                ROS_INFO("PENDING");
            }
            if (client->getState() == actionlib::SimpleClientGoalState::PENDING)
            {
                ROS_INFO("PENDING");
            }
            if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
            {
                ROS_INFO("PREEMPTED");
            }
            if (client->getState() == actionlib::SimpleClientGoalState::LOST)
            {
                ROS_INFO("LOST");
            }
        
        
    }


    std::pair<int,actionlib::SimpleClientGoalState> get_state()
    {
        return std::make_pair(now_task_ID,client->getState());
    }
    
    bool multi_point_cruise(const int ID,const std::vector<goal_point> &v)
    {
        if ( v.size() )
        {
            if ( now_task_ID != ID )
                cruise_num = 0;
            
            if ( cruise_num + 1 <= v.size() - 1 )
            {
                cruise_num++;
                return Navigator_task(ID,v[cruise_num]);
            }
            else if ( cruise_num == v.size() - 1 )
            {
                cruise_num = 0;
                return Navigator_task(ID,v[cruise_num]);
            }
        }
        return false;
        
    }
    
    bool simple_cruise(const int ID,const goal_point &point,const goal_point &point2)
    {
        if ( now_task_ID != ID )
        {
            cruise_num = 1;
        }
        if ( cruise_num == 1)
        {
            cruise_num = 2;
            return Navigator_task(ID,point);
            
        }else if ( cruise_num == 2)
        {
            cruise_num = 1;
            return Navigator_task(ID,point2);
        }
        return false;
    }

    bool Navigator_task(const int ID,const goal_point &point)
    {
        //debug_clientstate();
        

        //判断机器人导航状态，若空闲则处理任何任务，若处于导航任务中则优先执行紧急等级更高的任务
        if ( ( client->getState() == actionlib::SimpleClientGoalState::ACTIVE && now_task_ID < ID ) ||
             ( client->getState() == actionlib::SimpleClientGoalState::LOST || client->getState() == actionlib::SimpleClientGoalState::PREEMPTED || client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) )
        {
            //判断是否需要取消先前任务
            if ( client->getState() == actionlib::SimpleClientGoalState::ACTIVE && now_task_ID < ID )
            {
                ROS_INFO("任务ID(%d)在抢占先前的任务(%d)",ID,now_task_ID);
                client->cancelAllGoals();
                ros::Duration( 0.5 ).sleep(); //通信存在延迟，若暂停则可能导致刚发送的指令也被取消
            }
            else
            {
                ROS_INFO("任务ID(%d)在空闲状态下处理",ID);
            }
            //更新任务ID
            now_task_ID = ID;

            //无效点则取消导航任务，有效点则执行导航任务
            if ( point.Invalid_target_point )
            {
                ROS_ERROR("%d 无效点",ID);
                if ( client->getState() == actionlib::SimpleClientGoalState::LOST || client->getState() == actionlib::SimpleClientGoalState::PREEMPTED || client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) 
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                //创建目标点消息并发送
                move_base_msgs::MoveBaseGoal Goal_msg;
                Goal_msg.target_pose.header.frame_id = "map";
                Goal_msg.target_pose.pose.position.x = point.x;
                Goal_msg.target_pose.pose.position.y = point.y;
                Goal_msg.target_pose.pose.orientation.z = point.Z;
                Goal_msg.target_pose.pose.orientation.w = point.W;
                client->sendGoal(Goal_msg);
                
            }

            ros::Duration( 0.5 ).sleep(); //同理，等待服务器接受消息

            //判断服务器在接受信息后状态
            if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE || client->getState() == actionlib::SimpleClientGoalState::PENDING)
            {
                ROS_INFO("发送成功");
                return true;
            }
            else if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("已经到达");
                return true;
            }else
            {
                ROS_ERROR("发送失败");
                return false;
            }
        }
        else
        {
            ROS_WARN("任务ID(%d)无法执行,已有更高或相同等级的任务正在执行 正在执行任务的ID :%d",ID,now_task_ID);
            debug_clientstate();
            return false;
        }  
    }
private:

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *client = nullptr;
int now_task_ID = -1;
int cruise_num;
ros::NodeHandle *nh_ = nullptr;
};

/*  状态类型
    PENDING,
    ACTIVE,
    RECALLED,
    REJECTED,
    PREEMPTED,
    ABORTED,
    SUCCEEDED,
    LOST    */

//全局数据
Navigator navigator;
std::vector<goal_point> task_point;
std::vector<goal_point> cruise_point;
std::pair<goal_point,goal_point> cruise_two_point;


void idcb(const std_msgs::Int8::ConstPtr ID_)
{
    //测试用
    ROS_INFO("接收到ID %d",ID_->data);
    switch (ID_->data)
    {
    case 0: navigator.Navigator_task(ID_->data,goal_point(6.54,-1.06));
        break;
    case 1: navigator.Navigator_task(ID_->data,goal_point(3.48,-0.36));
        break;
    case 2: navigator.Navigator_task(ID_->data,goal_point(0.74,-1.22));
        break;
    case 3: navigator.Navigator_task(ID_->data,goal_point(-2.10994,2.45623));
        break;
    default:
        break;
    }
}


void task_performer(const int &i)
{
    if (navigator.Navigator_task(i,task_point[i-1]))
        {
            ROS_WARN("ID :(%d) TRUE",i);
        }
        else
        {
            if (!(navigator.get_state().second == actionlib::SimpleClientGoalState::ACTIVE || navigator.get_state().second == actionlib::SimpleClientGoalState::PENDING))
            {
                ROS_WARN("ID :(%d) FLASE",i);
            }
        }
}


void referee_msgs_cb(const qidian_tactics::driver::ConstPtr data)
{
    ROS_INFO("\033[2J\033[H");
    ROS_INFO("--------------------");
    ROS_INFO("比赛开始后 %d min %.1f sec", (int)(data->header.stamp.toSec() - data->start_time.toSec())/60,
                                               fmod(data->header.stamp.toSec() - data->start_time.toSec(), 60.0));
    
    
    
    // 任务ID (6) 若前方有敌人则终止导航 ;
    if ( data->detect_enemy )
    {
        task_performer(6);
        return;
    }

    // 任务ID (5) 当基地血量低于500时 或 比赛4分10秒（4*60+10）后 ；
    if ( data->own_base_HP <= 500 || ( data->header.stamp.toSec() - data->start_time.toSec() ) > 4 * 60 + 10 )
    {
        task_performer(5);
        return;
    }

    // 任务ID (4) 若血量不满，则在4分钟时到达补给点加血 ；
    if ( data->own_robot_HP < data->maximum_HP && ( data->header.stamp.toSec() - data->start_time.toSec() ) > 4 * 60 - 10 )
    {
        task_performer(4);
        return;
    }

    // 任务ID (3) 若我方基地受到首次攻击后，我方血量大于750 ；
    if ( data->own_base_HP >  750 && ( data->own_base_HP <  1500 || data->Virtual_Shield_Status != 100 ) )
    {
        task_performer(3);
        return;
    }

    // 任务ID (2) 若我方基地血量小于750 ；
    if ( data->own_base_HP <= 750 )
    {
        task_performer(2);
        return;
    }

    // 任务ID (1) 若中心增益点可占领则前往，（1min后首次可占领状态） ； 
    if ( data->Central_area_status == 1 || (( data->header.stamp.toSec() - data->start_time.toSec() ) > 50 && ( data->header.stamp.toSec() - data->start_time.toSec() ) < 60*1+10) )
    {
        task_performer(1);
        return;
    }

    // 任务ID (0) 两点来回巡航寻找敌人 ；
    if (!navigator.simple_cruise(0,cruise_two_point.first,cruise_two_point.second))
    {
        if (!(navigator.get_state().second == actionlib::SimpleClientGoalState::ACTIVE || navigator.get_state().second == actionlib::SimpleClientGoalState::PENDING))
        {
            ROS_ERROR("巡航失败 当前ID: %d",navigator.get_state().first);
            navigator.debug_clientstate();
        }
    }
    
}

/*
任务ID (6) 若前方有敌人则终止导航 ；
任务ID (5) 当基地血量低于500时 或 比赛4分10秒（4*60+10）后 ；目标点（3）
任务ID (4) 若血量不满，则在4分钟时到达补给点加血 ；目标点 （6）
任务ID (3) 若我方基地受到首次攻击后，我方血量大于750 ；目标点 （1）
任务ID (2) 若我方基地受到首次攻击后，我方血量小于750 ；目标点 （3）
任务ID (1) 若中心增益点可占领则前往，（1min后首次可占领状态） ； 目标点（2）
任务ID (0) 两点来回巡航寻找敌人 ；目标点（4、5）
*/

//从字符串从截取数值
float get_nums(int &i,std::string str)
{
    int a;
    for (a = 1; (i+a+1) < str.length() && str[i+a+1]!=',';){a++;}
    std::string son_str = str.substr(i+1,a);
    i+=a+1;
    return std::stof(son_str);
}
//字符串信息转坐标数组
void str_to_vector(std::string str, std::vector<goal_point> *v )
{
    //字符串转换成容器数据
    int i,a;
    for(i = 0;str[i+2]!='\0';i++)
    {
        if(str[i]=='(')
        {
            goal_point temp;
            temp.x=get_nums(i,str);
            temp.y=get_nums(i,str);
            temp.Z=get_nums(i,str);
            temp.W=get_nums(i,str);
            temp.Invalid_target_point = false;
            v->push_back(temp);
        }
    }
}


int main(int argc, char  *argv[])
{
    //注意：若move_base服务器重启，该节点也应重启，重新连接服务器
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tactics");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client(nh, "move_base", true);
    navigator.Setptr(&client,&nh);
    
    ros::Subscriber sub_id = nh.subscribe<std_msgs::Int8>("test_ID",1,idcb);
    ros::Subscriber sub_referee_msgs = nh.subscribe<qidian_tactics::driver>("/referee_msgs",10,referee_msgs_cb);
    
    //初始化目标坐标点

    str_to_vector(nh.param("/tactics/Task_0_point",std::string("(0,0,0,0),(0,0,0,0)")),&cruise_point);
    cruise_two_point.first = cruise_point[0];cruise_two_point.second = cruise_point[1];cruise_point.clear();
    
    str_to_vector(nh.param("/tactics/Task_1_point",std::string("(0,0,0,0)")),&task_point);
    str_to_vector(nh.param("/tactics/Task_2_point",std::string("(0,0,0,0)")),&task_point);
    str_to_vector(nh.param("/tactics/Task_3_point",std::string("(0,0,0,0)")),&task_point);
    str_to_vector(nh.param("/tactics/Task_4_point",std::string("(0,0,0,0)")),&task_point);
    str_to_vector(nh.param("/tactics/Task_5_point",std::string("(0,0,0,0)")),&task_point);
    task_point.push_back(goal_point());

    //测试输出
    ROS_INFO("%.2f %.2f %.2f %.2f",cruise_two_point.first.x,cruise_two_point.first.y,cruise_two_point.first.Z,cruise_two_point.first.W);
    ROS_INFO("%.2f %.2f %.2f %.2f",cruise_two_point.second.x,cruise_two_point.second.y,cruise_two_point.second.Z,cruise_two_point.second.W);
    
    std::for_each(task_point.begin(),task_point.end(),[](const goal_point point){
    ROS_INFO("%.2f %.2f %.2f %.2f",point.x,point.y,point.Z,point.W);
    }
    );

    ros::spin();
    return 0;
}
