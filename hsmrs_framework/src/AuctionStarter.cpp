#include "ros/ros.h"
#include <hsmrs_framework/TaskMsg.h>

class AuctionStarter
{
public:
    AuctionStarter()
    {
        ros::NodeHandle n;
        
        ros::Publisher bidPub = n.advertise<hsmrs_framework::TaskMsg>("/hsmrs/new_task", 100);
        
        hsmrs_framework::TaskMsg task = hsmrs_framework::TaskMsg();
        task.id = 0;
        task.parent_id = -1;
        task.priority = 10;
        task.type = "MyTask";
        
        while(n.ok())
        {
            ros::spinOnce();
            ROS_INFO("sending task\n");
            bidPub.publish(task);
            ROS_INFO("task sent\n");
            
            ros::Duration(5).sleep();
        }
    }  
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "AuctionStarter");

	AuctionStarter* robot = new AuctionStarter();
	return 0;
}
