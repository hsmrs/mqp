#include "ros/ros.h"
#include <hsmrs_framework/BidMsg.h>

class AuctionStarter
{
public:
    AuctionStarter()
    {
        ros::NodeHandle n;
        
        ros::Publisher bidPub = n.advertise<hsmrs_framework::BidMsg>("/hsmrs/auction", 100);
        
        hsmrs_framework::BidMsg bid = hsmrs_framework::BidMsg();
        bid.name = "AuctionStarter";
        bid.utility = 0;
        bid.task.id = 0;
        bid.task.parent_id = -1;
        bid.task.priority = 10;
        bid.task.type = "MyTask";
        
        while(n.ok())
        {
            ros::spinOnce();
            ROS_INFO("sending bid\n");
            bidPub.publish(bid);
            ROS_INFO("bid sent\n");
            
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
