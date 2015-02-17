#include <hsmrs_framework/Robot.h>
#include <hsmrs_framework/BidMsg.h>
#include "ros/ros.h"
#include <hsmrs_implementations/MyUtilityHelper.h>
#include <std_msgs/String.h>
#include <hsmrs_implementations/MyTask.h>
#include <hsmrs_implementations/MyAgentState.cpp>
#include <hsmrs_implementations/MyTaskList.h>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

boost::mutex atMutex;
boost::mutex listMutex;

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

	ros::Publisher bidPub;
	ros::Publisher claimPub;
	
	ros::Subscriber bidSub;
	ros::Subscriber newTaskSub;
	ros::Subscriber claimSub;
	
	MyTaskList taskList;
	MyUtilityHelper utiHelp;
	MyAgentState state;
	
	std::string name;
	std::string AUCTION_TOPIC;
	std::string NEW_TASK_TOPIC;
	std::string CLAIM_TOPIC;
public:
    BidderBot()
    {
        auctionList = std::map<int, AuctionTracker>();
        
        utiHelp = MyUtilityHelper();
        
        ros::NodeHandle n;

		AUCTION_TOPIC = "/hsmrs/auction";
		NEW_TASK_TOPIC = "/hsmrs/new_task";
		CLAIM_TOPIC = "/hsmrs/claim";
		name = "BidderBot";

		bidPub = n.advertise<hsmrs_framework::BidMsg>(AUCTION_TOPIC, 100);
		claimPub = n.advertise<hsmrs_framework::BidMsg>(CLAIM_TOPIC, 100);
		
		bidSub = n.subscribe(AUCTION_TOPIC, 1000, &BidderBot::handleBids, this);
		newTaskSub = n.subscribe(NEW_TASK_TOPIC, 100, &BidderBot::handleNewTask, this);
		claimSub = n.subscribe(CLAIM_TOPIC, 100, &BidderBot::handleClaims, this);
		
		state = MyAgentState();
		
		taskList = MyTaskList();
		
		ROS_INFO("starting up");
		
		ros::AsyncSpinner spinner(4);
		spinner.start();
		ros::waitForShutdown();
    }
    
    BidderBot(int id)
    {
        auctionList = std::map<int, AuctionTracker>();
        
        utiHelp = MyUtilityHelper();
        
        ros::NodeHandle n;

		AUCTION_TOPIC = "/hsmrs/auction";
		NEW_TASK_TOPIC = "/hsmrs/new_task";
		CLAIM_TOPIC = "/hsmrs/claim";

		bidPub = n.advertise<hsmrs_framework::BidMsg>(AUCTION_TOPIC, 100);
		claimPub = n.advertise<hsmrs_framework::BidMsg>(CLAIM_TOPIC, 100);
		
		bidSub = n.subscribe(AUCTION_TOPIC, 1000, &BidderBot::handleBids, this);
		newTaskSub = n.subscribe(NEW_TASK_TOPIC, 100, &BidderBot::handleNewTask, this);
		claimSub = n.subscribe(CLAIM_TOPIC, 100, &BidderBot::handleClaims, this);
		
		state = MyAgentState();
        state.setAttribute("attr1", (double)id);
		
		taskList = MyTaskList();
		
		ROS_INFO("starting up");
		
        name = "BidderBot " + std::to_string(id);
        ROS_INFO("id %d, name %s", id, getName().c_str());
        
		ros::AsyncSpinner spinner(4);
		spinner.start();
		ros::waitForShutdown();
    }

    void handleNewTask(const hsmrs_framework::TaskMsg::ConstPtr& msg)
    {
        ROS_INFO("got new task!\n");
        boost::mutex::scoped_lock atLock(atMutex);
        boost::mutex::scoped_lock listLock(listMutex);
        int id = msg->id;
        if(taskList.getTask(id) == NULL)
        {
            std::string type = msg->type;
            
            AuctionTracker at = AuctionTracker();
            double myBid = bid(msg);
            at.topBidder = getName();
            at.topUtility = myBid;
            at.haveBidded = true;
            auctionList[id] = at;
            atLock.unlock();
            
            if(type == "MyTask")
            {
                taskList.addTask(new MyTask(msg->id, msg->priority));
                listLock.unlock();
            }
            else
            {
                ROS_ERROR("unrecognized task type %s", type.c_str());
            }
            
            //spawn claimer thread
            boost::thread claimer = boost::thread(&BidderBot::claim, this, *msg, id, myBid);
            claimer.detach();
        }
        else
        {
            ROS_INFO("task with ID %d is not unique!\n", id);
        }
    }

	void handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg)
    {    
        int id = msg->task.id;
        std::string type = msg->task.type;
        std::string bidder = msg->name;
        double utility = msg->utility;
        
        ROS_INFO("got bid from %s\n", bidder.c_str());
        
        boost::mutex::scoped_lock atLock(atMutex);
        boost::mutex::scoped_lock listLock(listMutex);
        if(auctionList.count(id) == 0)
        {
            //track the auctioning of this task
            AuctionTracker at = AuctionTracker();
            double myBid = bid(msg);
            at.topBidder = (myBid > utility) ? getName() : bidder;
            at.topUtility = (myBid > utility) ? myBid : utility;
            at.haveBidded = true;
            auctionList[id] = at;
            atLock.unlock();
            
            //add to task list
            if(type == "MyTask")
            {
                taskList.addTask(new MyTask(msg->task.id, msg->task.priority));
                listLock.unlock();
            }
            else
            {
                ROS_ERROR("unrecognized task type %s", type.c_str());
            }
            //spawn claimer thread
            boost::thread claimer = boost::thread(&BidderBot::claim, this, msg->task, id, myBid);
            claimer.detach();
        }
        else
        {
            AuctionTracker* at = &(auctionList[id]);
            at->topBidder = (at->topUtility > utility) ? at->topBidder : bidder;
            at->topUtility = (at->topUtility > utility) ? at->topUtility : utility;
        }

    }
    
    void handleClaims(const hsmrs_framework::BidMsg::ConstPtr& msg)
    {
        int id = msg->task.id;
        std::string owner = msg->name;
        if(owner == getName()) return;
        boost::mutex::scoped_lock listLock(listMutex);
        taskList.getTask(id)->addOwner(owner);
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
    
    double bid(const hsmrs_framework::TaskMsg::ConstPtr& msg)
    {
        hsmrs_framework::BidMsg myBid = hsmrs_framework::BidMsg();
        myBid.task = *msg;
        myBid.name = getName();
        std::string type = msg->type;
        
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
    
    void claim(hsmrs_framework::TaskMsg taskMsg, int id, double myBid)
    {
        //sleep on it and decide whether to claim
        ROS_INFO("sleepytime");
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
        boost::mutex::scoped_lock atLock(atMutex);
        boost::mutex::scoped_lock listLock(listMutex);
        
        AuctionTracker at = auctionList[id];
        if(at.topBidder == getName())
        {
            ROS_INFO("claiming task %d", id);
            hsmrs_framework::BidMsg claimMsg = hsmrs_framework::BidMsg();
            claimMsg.name = getName();
            claimMsg.utility = myBid;
            claimMsg.task = taskMsg;
            claimPub.publish(claimMsg);
            
            at.taskClaimed = true;
            auctionList[id] = at;
            taskList.getTask(id)->addOwner(getName());
        }
    }
    
    std::string getName()
    {
        return name;
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
        state.getAttribute(attr) != NULL;
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
    int id = 0;
    ROS_INFO("%d", argc);
    if(argc == 2)
    {
        id = std::stoi(std::string(argv[1]));
    }
	ros::init(argc, argv, "BidderBot" + std::to_string(id));

	BidderBot* robot = new BidderBot(id);
	return 0;
}

