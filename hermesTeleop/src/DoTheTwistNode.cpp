#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

class DoTheTwistNode
{
public:
    DoTheTwistNode()
    {
        ros::NodeHandle n;
        
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("hermes/cmd_vel_mux/input/teleop", 10);
        
        geometry_msgs::Twist forward = geometry_msgs::Twist();
        
        //forward
        //forward.linear.y = 1;
        
        //rotate
        forward.angular.z = -1;
        
        geometry_msgs::Twist stop = geometry_msgs::Twist();
        
        bool flag = false;
        int count = 0;
        ros::Rate r = ros::Rate(30);
        while(n.ok())
        {
            count++;
            if(flag) pub.publish(forward);
            if(!flag) pub.publish(stop);
            if(count%30 == 0) flag = !flag;
            ros::spinOnce();
            r.sleep();
        }
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "twister");
    
    DoTheTwistNode twister = DoTheTwistNode();
    
    return 0;
}
