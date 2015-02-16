#include <hsmrs_framework/Robot.h>
#include <hsmrs_framework/BidMsg.h>
#include "ros/ros.h"
#include <hsmrs_implementations/MyUtilityHelper.h>
#include <std_msgs/String.h>
#include <hsmrs_implementations/MyTask.h>
#include <hsmrs_implementations/MyAgentState.cpp>
#include <hsmrs_implementations/MyTaskList.h>

class AuctionTracker
{
private:

public:
    std::string topBidder;
    double topUtility;
    bool haveBidded;
    bool taskClaimed;
    
    AuctionTracker()
    {
        topBidder = "";
        topUtility = -1;
        haveBidded = false;
        taskClaimed = false;
    }
};

class BidderBot: public Robot
{
private:
	std::map<int, AuctionTracker> auctionList;
	MyTaskList taskList;
	ros::Publisher bidPub;
	ros::Subscriber bidSub;
	MyUtilityHelper utiHelp;
	MyAgentState state;
	std::string AUCTION_TOPIC;
	std::string NEW_TASK_TOPIC;
public:
    BidderBot()
    {
        auctionList = std::map<int, AuctionTracker>();
        
        utiHelp = MyUtilityHelper();
        
        ros::NodeHandle n;

		ros::AsyncSpinner spinner(1);
		spinner.start();

		AUCTION_TOPIC = "/hsmrs/auction";

		bidPub = n.advertise<hsmrs_framework::BidMsg>(AUCTION_TOPIC, 100);
		
		bidSub = n.subscribe(AUCTION_TOPIC, 1000, &BidderBot::handleBids, this);
		
		state = MyAgentState();
		
		ros::spin();
    }

	void handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg)
    {
        ROS_INFO("got bid!\n");
    
        int id = msg->task.id;
        std::string type = msg->task.type;
        std::string bidder = msg->name;
        double utility = msg->utility;
        
        if(auctionList.count(id) == 0)
        {
            AuctionTracker at = AuctionTracker();
            double myBid = bid(msg);
            at.topBidder = (myBid > utility) ? getName() : bidder;
            at.topUtility = (myBid > utility) ? myBid : utility;
            at.haveBidded = true;
            auctionList[id] = at;
        }
        else
        {
            AuctionTracker* at = &(auctionList[id]);
            at->topBidder = (at->topUtility > utility) ? at->topBidder : bidder;
            at->topUtility = (at->topUtility > utility) ? at->topUtility : utility;
        }
    }
    
    double bid(const hsmrs_framework::BidMsg::ConstPtr& msg)
    {
        hsmrs_framework::BidMsg myBid = hsmrs_framework::BidMsg(*msg);
        myBid.name = getName();
        std::string type = msg->task.type;
        
        ROS_INFO("calculating utility...\n");
        if(type == "MyTask")
        {
            myBid.utility = utiHelp.calculate(this, new MyTask(0, 1));
        }
        else
        {
            myBid.utility = 0;
        }
        
        ROS_INFO("my utility is %f, publishing\n", myBid.utility);
        
        bidPub.publish(myBid);
        return myBid.utility;
    }
    
    std::string getName()
    {
        return std::string("BidderBot");
    }
    
    void cancelTask()
    {
    
    }
    
    void claimTask(Task* task)
    {
    
    }
    
    double getAttribute(std::string name)
    {
        return state.getAttribute(name);
    }
    
    double getUtility(Task* task)
    {
        return utiHelp.calculate(this, task);
    }
    
    AgentState* getState()
    {
        return new MyAgentState(state);
    }
    
    bool hasAttribute(std::string attr)
    {
        return false;
    }
    
    void setTask(Task* task)
    {
    
    }
    
    void verifyTaskClaim()
    {
    
    }
    
    void executeTask()
    {
    
    }
    
    void requestTaskForQueue(Task* task)
    {
    
    }
    
    void callForHelp()
    {
    
    }
    
    void handleTeleop()
    {
    
    }
    
    void sendMessage(std::string msg)
    {
    
    }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "BidderBot");

	BidderBot* robot = new BidderBot();
	return 0;
}

