#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/Int8.h"
class goal_point
{
public:
    goal_point(){}
    goal_point(const float &x,const float &y)
    {
        //position
        this->x = x;
        this->y = y;
        //orientation
        this->Z = 0;
        this->W = 1;
    }
        goal_point(const float &x,const float &y,const float &Z,const float &W)
    {
        //position
        this->x = x;
        this->y = y;
        //orientation
        this->Z = Z;
        this->W = W;
    }
    float x;
    float y;
    float Z;
    float W;
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

    bool Navigator_task(const int ID,const goal_point &point)
    {
        debug_clientstate();

        //判断机器人导航状态，若空闲则处理任何任务，若处于导航任务中则优先执行紧急等级更高的任务
        if ( ( client->getState() == actionlib::SimpleClientGoalState::ACTIVE && now_task_ID < ID ) ||
             ( client->getState() == actionlib::SimpleClientGoalState::LOST || client->getState() == actionlib::SimpleClientGoalState::PREEMPTED || client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) )
        {

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
            
            now_task_ID = ID;
            move_base_msgs::MoveBaseGoal Goal_msg;
            Goal_msg.target_pose.header.frame_id = "map";
            Goal_msg.target_pose.pose.position.x = point.x;
            Goal_msg.target_pose.pose.position.y = point.y;
            Goal_msg.target_pose.pose.orientation.z = point.Z;
            Goal_msg.target_pose.pose.orientation.w = point.W;
            client->sendGoal(Goal_msg);

            ros::Duration( 0.5 ).sleep(); //同理，等待服务器接受消息

            if (client->getState() == actionlib::SimpleClientGoalState::ACTIVE || client->getState() == actionlib::SimpleClientGoalState::PENDING)
            {
                ROS_INFO("发送成功");
                return true;
            }
            else if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("发送成功");
                return true;
            }else
            {
                ROS_ERROR("发送失败");
                return false;
            }
        }
        else
        {
            ROS_WARN("任务ID(%d)无法执行,已有更高或相同等级的任务正在执行",ID);
            return false;
        }  
    }
private:

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *client;
int now_task_ID;
ros::NodeHandle *nh_;
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

Navigator navigator;


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


int main(int argc, char  *argv[])
{
    //注意：若move_base服务器重启，该节点也应重启，重新连接服务器
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tactics");
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client(nh, "move_base", true);
    navigator.Setptr(&client,&nh);
    ros::Subscriber sub_id = nh.subscribe<std_msgs::Int8>("test_ID",1,idcb);
    ros::Time time;
    ros::spin();
    return 0;
}