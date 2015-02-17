#include "hulk/hulk_node.h"

/**
 * Begins the Robot's execution of its current Task.
 */
void Hulk::executeTask() {
	//std::string taskType = typeid(p_currentTask).name();
	std::string taskType = p_currentTask->getType();
	Behavior* behavior;

	GoToTask task;

	if (taskType == "GoToTask"){ 
		GoToTask* task = dynamic_cast<GoToTask*>(p_currentTask);
		behavior = new GoToBehavior(this, task->getGoal(), n);
	}
	else if(taskType == "FollowTagTask"){
		FollowTagTask* task = dynamic_cast<FollowTagTask*>(p_currentTask);
		//FollowTagBehavior ftb(this, 0.3, 0.5, task->getTagID(), n, VEL_TOPIC, LASER_TOPIC);
		//behavior = &ftb;
		behavior = new FollowTagBehavior(this, 0.3, 0.5, task->getTagID(), n, VEL_TOPIC, LASER_TOPIC);
	}
	p_currentBehavior = behavior;
	behavior->execute();
	setStatus("Busy");
}

void Hulk::pauseTask(){
	if (p_currentBehavior != NULL) p_currentBehavior->pause();
}

void Hulk::resumeTask(){
	if (p_currentBehavior != NULL) p_currentBehavior->resume();	
}

void Hulk::doGoToTask(double x, double y){
	/*MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Go To task completed");
		//sendLog("Go To task completed");
	}
	else
	{
		ROS_INFO("Go To task failed, asking for helps");
		//sendLog("Go To task failed, asking for helps");
		callForHelp();
	} */
}

/**
 * Request for the given Task to be sent to the TaskList
 * @param task The task to be queued
 */
void Hulk::requestTaskForQueue(Task* task) {

}


/**
 * Send a help request to the Human supervisor.
 */
void Hulk::callForHelp() {
	std_msgs::String msg;
	msg.data = "true";
	help_pub.publish(msg);
}

void Hulk::handleTeleop(){

}

void Hulk::registerWithGUI() {
	//name;requestTopic;logTopic;imageTopic;poseTopic;statusTopic;helpTopic;teleOpTopic

	std_msgs::String msg;
	std::stringstream ss;

	ss << NAME << ";" << REQUEST_TOPIC << ";" << LOG_TOPIC << ";" << IMAGE_TOPIC << ";"
			<< POSE_TOPIC << ";" << STATUS_TOPIC << ";" << HELP_TOPIC << ";" 
			<< TELE_OP_TOPIC;
	msg.data = ss.str();
	registration_pub.publish(msg);
}

void Hulk::sendMessage(std::string message){
	std_msgs::String msg;
	msg.data = message;
	log_pub.publish(msg);
}

void Hulk::requestCallback(const std_msgs::String::ConstPtr& msg)
{
	std::string request = msg->data;

	if (request == "tele-op"){
		pauseTask();
		setStatus("Tele-Op");
	}
	else if (request == "stop tele-op"){
		setStatus("Idle");
		resumeTask();
	}
	else if (request == "stop help"){

	}
}

void Hulk::teleOpCallback(const geometry_msgs::Twist::ConstPtr& msg){
	double linearVel = msg->linear.x * linearSpeed;
	double angularVel = msg->linear.y * angularSpeed;

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = linearVel;
	velMsg.angular.z = angularVel;

	vel_pub.publish(velMsg);
}

void Hulk::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	int bumper = msg->bumper;
	int state = msg->state;

	std::string bumperStr;
	std::string stateStr;
	if (bumper == msg->LEFT){
		bumperStr = "left";
	}
	else if (bumper == msg->RIGHT){
		bumperStr = "right";
	}
	else if (bumper == msg->CENTER){
		bumperStr = "center";
	}

	if (state == msg->RELEASED){
		stateStr = "released";
		callForHelp();
	}
	else if (state == msg->RELEASED){
		stateStr = "pressed";
	}

	std::string message = "My " + bumperStr + " bumper was " + stateStr + "!";
	sendMessage(message);

}
/* //TODO do something about GUI task assignment (i.e. this callback)
void Hulk::newTaskCallback(const std_msgs::String::ConstPtr& msg){
	std::string data = msg->data;

	std::vector<std::string> items;
	std::string delimiter = ";";
	size_t pos = 0;

	while ((pos = data.find(delimiter)) != std::string::npos) {
    	items.push_back(data.substr(0, pos));
		data.erase(0, pos + delimiter.length());
	}

	std::string type = items[1];
	Task* task;
	if (type == "GoTo"){
		task = new GoToTask(msg->data);
	}
	else if (type == "FollowTag"){
		task = new FollowTagTask(msg->data);
	}
	else{ //Task not recognized
		return;
	}
	std::vector<std::string> owners = task->getOwners();
	
	if (std::find(owners.begin(), owners.end(), NAME)!=owners.end()){
		claimTask(task);
	} else{
		taskList->addTask(task);
	}
}
*/

//TODO this needs to be toggleable
void Hulk::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
	if (msg->markers.size() == 0) return;

	for (int i = 0; i < msg->markers.size(); ++i)
	{
		if (msg->markers[i].id == 0)
		{
		    //TODO filter this
		    state->setAttribute("distance", msg->markers[0].pose.pose.position.x);
		}
	}
}

void Hulk::updatedTaskCallback(const std_msgs::String::ConstPtr& msg){

}

void Hulk::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

}

Hulk::Hulk() : NAME("Hulk"), REGISTRATION_TOPIC("hsmrs/robot_registration"), IMAGE_TOPIC("hulk/camera/rgb/image_mono"), 
		LOG_TOPIC("hulk/log_messages"), STATUS_TOPIC("hulk/status"), HELP_TOPIC("hulk/help"), POSE_TOPIC("hulk/pose"),
		REQUEST_TOPIC("hulk/requests"), TELE_OP_TOPIC("hulk/tele_op"), VEL_TOPIC("hulk/cmd_vel_mux/input/teleop"),
		BUMPER_TOPIC("/hulk/mobile_base/events/bumper"), NEW_TASK_TOPIC("/hsmrs/new_task"), 
		UPDATED_TASK_TOPIC("/hsmrs/updated_task_topic"), LASER_TOPIC("hulk/scan"), MARKER_TOPIC("/hulk/ar_pose_marker"),
		AUCTION_TOPIC("/hsmrs/auction"), CLAIM_TOPIC("/hsmrs/claim")
		{
	ROS_INFO("Hulk initializing");
	taskList = new MyTaskList();
	utiHelp = new MyUtilityHelper();
	state = new MyAgentState();
	state->setAttribute("speed", 1);
	state->setAttribute("distance", 1000);
	
    auctionList = std::map<int, AuctionTracker>();

	linearSpeed = 0.3;
	angularSpeed = 0.8;

	//GUI Publishers and subscribers
	registration_pub = n.advertise<std_msgs::String>(REGISTRATION_TOPIC, 100);
	log_pub = n.advertise<std_msgs::String>(LOG_TOPIC, 100);
	status_pub = n.advertise<std_msgs::String>(STATUS_TOPIC, 100);
	help_pub = n.advertise<std_msgs::String>(HELP_TOPIC, 100);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 100);

	//new_task_sub = n.subscribe(NEW_TASK_TOPIC, 1000, &Hulk::newTaskCallback, this);
	updated_task_sub = n.subscribe(UPDATED_TASK_TOPIC, 1000, &Hulk::updatedTaskCallback, this);
	request_sub = n.subscribe(REQUEST_TOPIC, 1000, &Hulk::requestCallback, this);
	teleOp_sub = n.subscribe(TELE_OP_TOPIC, 1000, &Hulk::teleOpCallback, this);

	//Turtlebot publishers and subscribers
	vel_pub = n.advertise<geometry_msgs::Twist>(VEL_TOPIC, 100);
	bumper_sub = n.subscribe(BUMPER_TOPIC, 1000, &Hulk::bumperCallback, this);
	//laser_sub = n.subscribe(LASER_TOPIC, 1000, &Hulk::laserCallback, this);
	tag_sub = n.subscribe(MARKER_TOPIC, 1000, &Hulk::tagCallback, this);

	bidPub = n.advertise<hsmrs_framework::BidMsg>(AUCTION_TOPIC, 100);
	claimPub = n.advertise<hsmrs_framework::BidMsg>(CLAIM_TOPIC, 100);
	
	bidSub = n.subscribe(AUCTION_TOPIC, 1000, &Hulk::handleBids, this);
	newTaskSub = n.subscribe(NEW_TASK_TOPIC, 100, &Hulk::handleNewTask, this);
	claimSub = n.subscribe(CLAIM_TOPIC, 100, &Hulk::handleClaims, this);


	//ros::spinOnce();
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ROS_INFO("Hulk running");

/*
	ros::Rate loop_rate(1);
	loop_rate.sleep();
*/
	registerWithGUI();

	while (ros::ok()){
		//Task* nextTask = taskList->pullNextTask();
		//if (nextTask != NULL){
			//bid(nextTask);
		//}
	}
	ros::waitForShutdown();
	//ros::spin();
	//while (ros::ok()) {
	//	loop_rate.sleep();
	//}
}

std::string Hulk::getName(){
	return NAME;
}

/**
 * Returns the value of the specified attribute from this Robot's AgentState.
 * @param attr The name of the attribute to get
 * @return The value of the attribute
 */
double Hulk::getAttribute(std::string name)
{
    return state->getAttribute(name);
}

/**
 * Returns this Robot's utility for the specified Task.
 * @param task A pointer to the task for which to get a utility.
 * @return This Robot's utility for the given Task.
 */
double Hulk::getUtility(Task* task)
{
    return utiHelp->calculate(this, task);
}

/**
 * Returns this Robot's AgentState.
 * @return The AgentState representing the state of this Robot.
 */
AgentState* Hulk::getState()
{
    return new MyAgentState(*state);
}

/**
 * Checks if this Robot has the given attribute.
 * @param attr The name of the target attribute
 * @return True if the robot has the named attribute.
 */
bool Hulk::hasAttribute(std::string attr)
{
    return state->getAttribute(attr) != NULL;
}

/**
 * Sets this Robot's currently active Task.
 * @param A pointer to the Task to be set
 */
void Hulk::setTask(Task* task) {
	p_currentTask = task;
}

/**
 * Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
 */
void Hulk::verifyTaskClaim() {

}

/**
 * Stops execution of the current Task and requests that
 * the Task be returned to the TaskList.
 */
void Hulk::cancelTask() {

}

/**
 * Asks the Agent to claim a task pointed to by \p task.
 * @param task A pointer to the task object to be claimed.
 */
void Hulk::claimTask(Task* task) {
	ROS_INFO("Claiming task!");
	p_currentTask = task;
	executeTask();
}

std::string Hulk::getStatus(){
	return status;
}

void Hulk::setStatus(std::string newStatus){
	status = newStatus;
	std_msgs::String msg;
	msg.data = status;
	status_pub.publish(msg);
}


void Hulk::handleNewTask(const hsmrs_framework::TaskMsg::ConstPtr& msg)
{
    ROS_INFO("got new task!\n");
    boost::mutex::scoped_lock atLock(atMutex);
    boost::mutex::scoped_lock listLock(listMutex);
    int id = msg->id;
    if(taskList->getTask(id) == NULL)
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
            taskList->addTask(new MyTask(msg->id, msg->priority));
            listLock.unlock();
        }
        else if(type == "FollowTagTask")
        {
            if(msg->param_values.size() > 0)
            {
                taskList->addTask(new FollowTagTask(msg->id, msg->priority, std::stoi(msg->param_values[0])));
            }
            else
            {
                taskList->addTask(new FollowTagTask(msg->id, msg->priority));
            }
        }
        else if(type == "GoToTask")
        {
            //TODO fill in
        }
        else
        {
            ROS_ERROR("unrecognized task type %s", type.c_str());
        }
        
        //spawn claimer thread
        boost::thread claimer = boost::thread(&Hulk::claimWorker, this, *msg, id, myBid);
        claimer.detach();
    }
    else
    {
        ROS_INFO("task with ID %d is not unique!\n", id);
    }
}

void Hulk::handleClaims(const hsmrs_framework::BidMsg::ConstPtr& msg)
{
    int id = msg->task.id;
    std::string owner = msg->name;
    if(owner == getName()) return;
    boost::mutex::scoped_lock listLock(listMutex);
    taskList->getTask(id)->addOwner(owner);
}

void Hulk::claimWorker(hsmrs_framework::TaskMsg taskMsg, int id, double myBid)
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
        taskList->getTask(id)->addOwner(getName());
    }
    
	p_currentTask = taskList->getTask(id);
	executeTask();
}


/**
 * Handles the auctioning of Tasks by sending and receiving bids.
 */
void Hulk::handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg)
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
            taskList->addTask(new MyTask(msg->task.id, msg->task.priority));
            listLock.unlock();
        }
        else if(type == "FollowTagTask")
        {
            if(msg->task.param_values.size() > 0)
            {
                taskList->addTask(new FollowTagTask(msg->task.id, msg->task.priority, std::stoi(msg->task.param_values[0])));
            }
            else
            {
                taskList->addTask(new FollowTagTask(msg->task.id, msg->task.priority));
            }
        }
        else if(type == "GoToTask")
        {
            //TODO fill in
        }
        else
        {
            ROS_ERROR("unrecognized task type %s", type.c_str());
        }
        //spawn claimer thread
        boost::thread claimer = boost::thread(&Hulk::claimWorker, this, msg->task, id, myBid);
        claimer.detach();
    }
    else
    {
        AuctionTracker* at = &(auctionList[id]);
        at->topBidder = (at->topUtility > utility) ? at->topBidder : bidder;
        at->topUtility = (at->topUtility > utility) ? at->topUtility : utility;
    }

}

/**
 * Makes this Robot bid on the given task
 */
double Hulk::bid(const hsmrs_framework::BidMsg::ConstPtr& msg)
{
    hsmrs_framework::BidMsg myBid = hsmrs_framework::BidMsg(*msg);
    myBid.name = getName();
    std::string type = msg->task.type;
    
    ROS_INFO("calculating utility...\n");
    if(type == "MyTask")
    {
        myBid.utility = utiHelp->calculate(this, new MyTask(0, 1));
    }
    else
    {
        myBid.utility = 0;
    }
    
    ROS_INFO("my utility is %f, publishing\n", myBid.utility);
    
    bidPub.publish(myBid);
    return myBid.utility;
}

double Hulk::bid(const hsmrs_framework::TaskMsg::ConstPtr& msg)
{
    hsmrs_framework::BidMsg myBid = hsmrs_framework::BidMsg();
    myBid.task = *msg;
    myBid.name = getName();
    std::string type = msg->type;
    
    ROS_INFO("calculating utility...\n");
    if(type == "MyTask")
    {
        myBid.utility = utiHelp->calculate(this, new MyTask(0, 1));
    }
    else
    {
        myBid.utility = 0;
    }
    
    ROS_INFO("my utility is %f, publishing\n", myBid.utility);
    
    bidPub.publish(myBid);
    return myBid.utility;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hulk");

	Hulk* robot = new Hulk();
	return 0;
}
