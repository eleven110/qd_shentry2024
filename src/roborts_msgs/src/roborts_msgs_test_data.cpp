#include <ros/ros.h>		//ros必备头文件
#include <roborts_msgs/RobotStatus.h>		//自己的msg头文件，test是你的包名，test.h是将
							  //你的msg文件的.msg后缀改成了.h
#include <roborts_msgs/LocationInfo.h>		//自己的msg头文件，test是你的包名，test.h是将
#include <roborts_msgs/GameStatus.h>		//自己的msg头文件，test是你的包名，test.h是将

#include <sstream>			 //这个是用来定义以下的std::stringstream的




void RobotStatus_ake(){

}

void GameStatus_fake(){

}

void LocationInfo_fake(){

}



int main(int argc,char **argv)
{
    ros::init(argc,argv,"test");				//test是你的节点名称
    ros::NodeHandle n;
    ros::Publisher RobotStatus_pub = n.advertise<roborts_msgs::RobotStatus>("RobotStatus",1000);
    ros::Publisher GameStatus_pub = n.advertise<roborts_msgs::GameStatus>("GameStatus",1000);
    ros::Publisher LocationInfo_pub = n.advertise<roborts_msgs::LocationInfo>("LocationInfo",1000);

    //上面这个句子注意了，advertise后边的<>里面放的是你想要发布的节点的
    //包名::msg文件名去掉后缀，里面是你所发布的主题的名称。1000你可以当作是一个
    //栈或者队列来看待，就是发布的消息超过1000的时候，他会开始舍弃发布过的信息
    //仅此而已，你也可以自己添加，换成1或者其他的都行
    
    ros::Rate loop_rate(10);		//设置发布的频率为10,可以自己更改

    //下面三个是我打算用来附给msg并发布出去的
 			
    std::stringstream ss("Hello World!");
    while(ros::ok())
    {
        //定义一个roborts_msgs::RobotStatus类型的msg变量。（也就是包名::msg文件名去掉后缀）
        roborts_msgs::RobotStatus msg;
        //下面三句我们可以清楚的看到我直接用了.a还有.b跟.c，其实就是我刚才
        //在msg文件下命名的名称，如果你是其他的名称，那么你就直接叫那个名字
        //就可以啦。
        msg.remain_HP = 50;
        msg.robot_id = 7;
        sleep(3);
        msg.remain_HP = 100;
        sleep(10);
        msg.remain_HP = 50;

        // sleep(50);
        msg.remain_HP = 200;

        RobotStatus_pub.publish(msg);		//这句就是最重要的发布消息啦。
		//最后两句跟ros::init都是固定要的，加上去就可以啦.
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
