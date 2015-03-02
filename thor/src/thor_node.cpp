#include "thor/thor_node.h"

/**
 * Begins the Robot's execution of its current Task.
 */
void Thor::executeTask() {
	ROS_INFO("Execute task!");
	boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
	//std::string taskType = typeid(p_currentTask).name();
	std::string taskType = p_currentTask->getType();
	Behavior* behavior;

	GoToTask task;

	if (taskType == "GoToTask"){ 
		GoToTask* task = dynamic_cast<GoToTask*>(p_currentTask);
		behavior = new GoToBehavior(this, task->getGoal().position, n);
	}
	else if(taskType == "FollowTagTask"){
		FollowTagTask* task = dynamic_cast<FollowTagTask*>(p_currentTask);
		//FollowTagBehavior ftb(this, 0.3, 0.5, task->getTagID(), n, VEL_TOPIC, LASER_TOPIC);
		//behavior = &ftb;
		behavior = new FollowTagBehavior(this, 0.3, 0.5, task->getTagID(), n, VEL_TOPIC, LASER_TOPIC);
	}
	else if(taskType == "SearchTask"){
		ROS_INFO("Executing Search task!");
		SearchTask* task = dynamic_cast<SearchTask*>(p_currentTask);
		behavior = new SearchBehavior(this, 0.3, 0.5, 1, task->getTagID(), task->getBoundaryVertices(), n, VEL_TOPIC);
	}
	p_currentBehavior = behavior;
	behavior->execute();
	currentTaskLock.unlock();
	setStatus("Busy");
}

void Thor::pauseTask(){
    boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
	if (p_currentBehavior != NULL) p_currentBehavior->pause();
	currentTaskLock.unlock();
}

void Thor::resumeTask(){
    boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
	if (p_currentBehavior != NULL) p_currentBehavior->resume();
	currentTaskLock.unlock();	
}

void Thor::doGoToTask(double x, double y){
}

/**
 * Request for the given Task to be sent to the TaskList
 * @param task The task to be queued
 */
void Thor::requestTaskForQueue(Task* task) {

}


/**
 * Send a help request to the Human supervisor.
 */
void Thor::callForHelp() {
	std_msgs::String msg;
	msg.data = "true";
	help_pub.publish(msg);
}

void Thor::handleTeleop(){

}

void Thor::registerWithGUI() {
	//name;requestTopic;logTopic;imageTopic;poseTopic;statusTopic;helpTopic;teleOpTopic

	std_msgs::String msg;
	std::stringstream ss;

	ss << NAME << ";" << NAME + "/" + REQUEST_TOPIC << ";" << NAME + "/" + LOG_TOPIC << ";" << NAME + "/" + IMAGE_TOPIC << ";"
			<< NAME + "/" + POSE_TOPIC << ";" << NAME + "/" + STATUS_TOPIC << ";" << NAME + "/" + HELP_TOPIC << ";" 
			<< NAME + "/" + TELE_OP_TOPIC;
	msg.data = ss.str();
	registration_pub.publish(msg);
}

void Thor::sendMessage(std::string message){
	std_msgs::String msg;
	msg.data = message;
	log_pub.publish(msg);
}

void Thor::requestCallback(const std_msgs::String::ConstPtr& msg)
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

void Thor::teleOpCallback(const geometry_msgs::Twist::ConstPtr& msg){
	double linearVel = msg->linear.x * linearSpeed;
	double angularVel = msg->linear.y * angularSpeed;

	geometry_msgs::Twist velMsg;
	velMsg.linear.x = linearVel;
	velMsg.angular.z = angularVel;

	vel_pub.publish(velMsg);
}

void Thor::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
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

//TODO this needs to be toggleable
void Thor::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
	if (msg->markers.size() == 0) return;

	//for (int i = 0; i < msg->markers.size(); ++i)
	//{
		//if (msg->markers[i].id == 0)
		//{
		    //TODO filter this
		    state->setAttribute("distance", msg->markers[0].pose.pose.position.x);
		//}
	//}
}

void Thor::updatedTaskCallback(const hsmrs_framework::TaskMsg::ConstPtr& msg){
    Task* old;
    Task* update;
    
    std::string type = msg->type;
    
    ROS_INFO("got task update for task %llu of type %s", msg->id, std::string(msg->type).c_str());
    
    if(type == "MyTask")
    {
        update = new MyTask(msg->id, msg->priority);
    }
    else if(type == "FollowTagTask")
    {
        if(msg->param_values.size() > 0)
        {
            update = new FollowTagTask(msg->id, msg->priority, std::stoi(msg->param_values[0]));
        }
        else
        {
            update = new FollowTagTask(msg->id, msg->priority);
        }
    }
    else if(type == "GoToTask")
    {
        update = new GoToTask(msg);
    }
    else if(type == "SearchTask")
    {
        update = new SearchTask(msg);
    }
    else
    {
        ROS_ERROR("unrecognized task type %s", type.c_str());
    }
    
    boost::mutex::scoped_lock listLock(listMutex);
    boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
    old = taskList->getTask(update->getID());
    std::vector<std::string> oldOwners = old->getOwners();
    
    if(std::find(oldOwners.begin(), oldOwners.end(), getName()) != oldOwners.end() && p_currentTask != NULL && p_currentBehavior != NULL) //if this is my task
    {
        ROS_INFO("I own this task");
        if(std::find(msg->owners.begin(), msg->owners.end(), getName()) == msg->owners.end() || msg->status == "complete" || msg->status == "deleted") //if I'm no longer working on it
        {
            ROS_INFO("task is complete/I'm no longer an owner, ending");
            p_currentTask = NULL;
            p_currentBehavior->stop();
            delete p_currentBehavior;
            p_currentBehavior = NULL;
        }
        else
        {
            ROS_INFO("task isn't complete, updating my copy");
            p_currentTask = update;
        }
    }
    
    //TODO make it so robots can be added
    
    ROS_INFO("updating task list copy");
    taskList->removeTask(update->getID());
    taskList->addTask(update);
    listLock.unlock();
    currentTaskLock.unlock();
    
    ROS_INFO("done updating");
}

void Thor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

}

void Thor::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	geometry_msgs::PoseStamped tempMsg;
	geometry_msgs::PoseStamped poseMsg;
	
	tempMsg.header.frame_id = NAME + "/odom_combined";
	tempMsg.header.stamp = ros::Time::now();
	tempMsg.pose = msg->pose.pose;
	try{
		listener.waitForTransform(NAME + "/odom_combined", "map",
			ros::Time::now(), ros::Duration(0.5));
		listener.transformPose("map", tempMsg, poseMsg);
	}
	catch (tf::TransformException &ex) {
		//ROS_ERROR("Error in Thor Pose Callback: %s",ex.what());
		return;
	}
	
	state->setAttribute("x", poseMsg.pose.position.x);
	state->setAttribute("y", poseMsg.pose.position.y);
	
	pose_pub.publish(poseMsg);
}

Thor::Thor(std::string name, double speed) : NAME(name), REGISTRATION_TOPIC("/hsmrs/robot_registration"), IMAGE_TOPIC("camera/rgb/image_mono"), 
		LOG_TOPIC("log_messages"), STATUS_TOPIC("status"), HELP_TOPIC("help"), POSE_TOPIC("pose"),
		REQUEST_TOPIC("requests"), TELE_OP_TOPIC("tele_op"), VEL_TOPIC("cmd_vel_mux/input/teleop"),
		BUMPER_TOPIC("mobile_base/events/bumper"), NEW_TASK_TOPIC("/hsmrs/new_task"), 
		UPDATED_TASK_TOPIC("/hsmrs/updated_task"), LASER_TOPIC("scan"), IN_POSE_TOPIC("odom_filter/odom_combined"),
		MARKER_TOPIC("ar_pose_marker"),
		AUCTION_TOPIC("/hsmrs/auction"), CLAIM_TOPIC("/hsmrs/claim"), TASK_PROGRESS_TOPIC("progress")
{

	taskList = new MyTaskList();
	utiHelp = new MyUtilityHelper();
	state = new MyAgentState();
	
	state->setAttribute("speed", speed);
	state->setAttribute("distance", 1000.0);
	state->setAttribute("x", 0);
	state->setAttribute("y", 0);

	linearSpeed = 0.3;
	angularSpeed = 0.8;

	ros::AsyncSpinner spinner(0);
	spinner.start();

	//GUI Publishers and subscribers
	registration_pub = n.advertise<std_msgs::String>(REGISTRATION_TOPIC, 100);
	log_pub = n.advertise<std_msgs::String>(LOG_TOPIC, 100);
	status_pub = n.advertise<std_msgs::String>(STATUS_TOPIC, 100);
	help_pub = n.advertise<std_msgs::String>(HELP_TOPIC, 100);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 100);
	updatedTaskPub = n.advertise<hsmrs_framework::TaskMsg>(UPDATED_TASK_TOPIC, 100);

	updated_task_sub = n.subscribe(UPDATED_TASK_TOPIC, 1000, &Thor::updatedTaskCallback, this);
	request_sub = n.subscribe(REQUEST_TOPIC, 1000, &Thor::requestCallback, this);
	teleOp_sub = n.subscribe(TELE_OP_TOPIC, 1000, &Thor::teleOpCallback, this);

	//Turtlebot publishers and subscribers
	vel_pub = n.advertise<geometry_msgs::Twist>(VEL_TOPIC, 100);
	bumper_sub = n.subscribe(BUMPER_TOPIC, 1000, &Thor::bumperCallback, this);
	pose_sub = n.subscribe(IN_POSE_TOPIC, 1, &Thor::poseCallback, this);
	//laser_sub = n.subscribe(LASER_TOPIC, 1000, &Thor::laserCallback, this);
	tag_sub = n.subscribe(MARKER_TOPIC, 1000, &Thor::tagCallback, this);

	bidPub = n.advertise<hsmrs_framework::BidMsg>(AUCTION_TOPIC, 100);
	claimPub = n.advertise<hsmrs_framework::BidMsg>(CLAIM_TOPIC, 100);
	
	bidSub = n.subscribe(AUCTION_TOPIC, 1000, &Thor::handleBids, this);
	newTaskSub = n.subscribe(NEW_TASK_TOPIC, 100, &Thor::handleNewTask, this);
	claimSub = n.subscribe(CLAIM_TOPIC, 100, &Thor::handleClaims, this);
	taskProgressSub = n.subscribe(TASK_PROGRESS_TOPIC, 100, &Thor::handleProgress, this);

	//ros::spinOnce();
	ros::Rate loop_rate(1);
	loop_rate.sleep();

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

std::string Thor::getName(){
	return NAME;
}


/**
 * Returns the value of the specified attribute from this Robot's AgentState.
 * @param attr The name of the attribute to get
 * @return The value of the attribute
 */
double Thor::getAttribute(std::string attr) {
    return state->getAttribute(attr);
}

/**
 * Returns this Robot's utility for the specified Task.
 * @param task A pointer to the task for which to get a utility.
 * @return This Robot's utility for the given Task.
 */
double Thor::getUtility(Task *task) {

}

/**
 * Returns this Robot's AgentState.
 * @return The AgentState representing the state of this Robot.
 */
AgentState* Thor::getState() {
    return new MyAgentState(*state);
}

/**
 * Checks if this Robot has the given attribute.
 * @param attr The name of the target attribute
 * @return True if the robot has the named attribute.
 */
bool Thor::hasAttribute(std::string attr) {
    try
    {
        state->getAttribute(attr);
        return true;
    }
    catch(...)
    {
        return false;
    }
}

/**
 * Sets this Robot's currently active Task.
 * @param A pointer to the Task to be set
 */
void Thor::setTask(Task* task) {
    boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
	p_currentTask = task;
	currentTaskLock.unlock();
}

/**
 * Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
 */
void Thor::verifyTaskClaim() {

}

/**
 * Stops execution of the current Task and requests that
 * the Task be returned to the TaskList.
 */
void Thor::cancelTask() {
    ROS_INFO("canceling task");
    boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
    ROS_INFO("got task lock");
    if(p_currentTask != NULL)
    {
        hsmrs_framework::TaskMsg* update = p_currentTask->toMsg();
        update->status = "complete";
        ROS_INFO("update message\n\tid:%llu\n\ttype:%s\n\tstatus:%s", update->id, update->type.c_str(), update->status.c_str());
        updatedTaskPub.publish(*update);
    }
    ROS_INFO("unlocking");
    currentTaskLock.unlock();
}

/**
 * Asks the Agent to claim a task pointed to by \p task.
 * @param task A pointer to the task object to be claimed.
 */
void Thor::claimTask(Task* task) {
	ROS_INFO("Claiming task!");
	boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
	p_currentTask = task;
	currentTaskLock.unlock();
	executeTask();
}

std::string Thor::getStatus(){
	return status;
}

void Thor::setStatus(std::string newStatus){
	status = newStatus;
	std_msgs::String msg;
	msg.data = status;
	status_pub.publish(msg);
}

void Thor::handleNewTask(const hsmrs_framework::TaskMsg::ConstPtr& msg)
{
    ROS_INFO("got new task!");
    boost::mutex::scoped_lock atLock(atMutex);
    boost::mutex::scoped_lock listLock(listMutex);
    int id = msg->id;
    if(taskList->getTask(id) == NULL)
    {
        std::string type = msg->type;
        Task* task;
        
        if(type == "MyTask")
        {
            task = new MyTask(msg->id, msg->priority);
            taskList->addTask(task);
        }
        else if(type == "FollowTagTask")
        {
            if(msg->param_values.size() > 0)
            {
                task = new FollowTagTask(msg->id, msg->priority, std::stoi(msg->param_values[0]));
            }
            else
            {
                task = new FollowTagTask(msg->id, msg->priority);
            }
            taskList->addTask(task);
        }
        else if(type == "GoToTask")
        {
            task = new GoToTask(msg);
            taskList->addTask(task);
        }
        else if(type == "SearchTask")
        {
            task = new SearchTask(msg);
            taskList->addTask(task);
        }
        else
        {
            ROS_ERROR("unrecognized task type %s", type.c_str());
            return;
        }
        
        listLock.unlock();
        
        double myBid = 0;
        
        if(msg->owners.size() < task->getMaxOwners())
        {
            AuctionTracker at = AuctionTracker();
            myBid = bid(msg);
            at.topBidder = getName();
            at.topUtility = myBid;
            at.haveBidded = true;
            auctionList[id] = at;
            atLock.unlock();
        }
        
        //spawn claimer thread
        boost::thread claimer = boost::thread(&Thor::claimWorker, this, *msg, id, myBid);
        claimer.detach();
    }
    else
    {
        ROS_INFO("task with ID %d is not unique!", id);
        Task* dupe = taskList->getTask(id);
        ROS_INFO("task with ID %d has type %s", id, dupe->getType().c_str());
    }
}

void Thor::handleClaims(const hsmrs_framework::BidMsg::ConstPtr& msg)
{
    int id = msg->task.id;
    std::string owner = msg->name;
    if(owner == getName()) return;
    boost::mutex::scoped_lock listLock(listMutex);
    taskList->getTask(id)->addOwner(owner);
}

void Thor::claimWorker(hsmrs_framework::TaskMsg taskMsg, int id, double myBid)
{
    //sleep on it and decide whether to claim
    ROS_INFO("sleepytime");
    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    boost::mutex::scoped_lock atLock(atMutex);
    boost::mutex::scoped_lock listLock(listMutex);
    
    AuctionTracker at = auctionList[id];
    if(at.topBidder == getName() || std::find(taskMsg.owners.begin(), taskMsg.owners.end(), getName()) != taskMsg.owners.end())
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
        
        boost::mutex::scoped_lock currentTaskLock(currentTaskMutex);
    	p_currentTask = taskList->getTask(id);
    	currentTaskLock.unlock();
	    executeTask();
    }
}


/**
 * Handles the auctioning of Tasks by sending and receiving bids.
 */
void Thor::handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg)
{    
    int id = msg->task.id;
    std::string type = msg->task.type;
    std::string bidder = msg->name;
    double utility = msg->utility;
    
    ROS_INFO("got bid from %s", bidder.c_str());
    
    boost::mutex::scoped_lock atLock(atMutex);
    boost::mutex::scoped_lock listLock(listMutex);
    if(auctionList.count(id) == 0)
    {
        std::string type = msg->task.type;
        Task* task;
        
        if(type == "MyTask")
        {
            task = new MyTask(msg->task.id, msg->task.priority);
            taskList->addTask(task);
        }
        else if(type == "FollowTagTask")
        {
            if(msg->task.param_values.size() > 0)
            {
                task = new FollowTagTask(msg->task.id, msg->task.priority, std::stoi(msg->task.param_values[0]));
            }
            else
            {
                task = new FollowTagTask(msg->task.id, msg->task.priority);
            }
            taskList->addTask(task);
        }
        else if(type == "GoToTask")
        {
            task = new GoToTask(msg->task);
            taskList->addTask(task);
        }
        else if(type == "SearchTask")
        {
            task = new SearchTask(msg->task);
            taskList->addTask(task);
        }
        else
        {
            ROS_ERROR("unrecognized task type %s", type.c_str());
            return;
        }
        
        listLock.unlock();
        
        double myBid = 0;
        
        if(msg->task.owners.size() < task->getMaxOwners())
        {
            AuctionTracker at = AuctionTracker();
            myBid = bid(msg);
            at.topBidder = getName();
            at.topUtility = myBid;
            at.haveBidded = true;
            auctionList[id] = at;
            atLock.unlock();
        }
        
        //spawn claimer thread
        boost::thread claimer = boost::thread(&Thor::claimWorker, this, msg->task, id, myBid);
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
double Thor::bid(const hsmrs_framework::BidMsg::ConstPtr& msg)
{
    hsmrs_framework::BidMsg myBid = hsmrs_framework::BidMsg(*msg);
    myBid.name = getName();
    std::string type = msg->task.type;
    
    if(p_currentTask == NULL)
    {
        ROS_INFO("calculating utility...");
        if(type == "MyTask")
        {
            myBid.utility = utiHelp->calculate(this, new MyTask(0, 1));
        }
	    else if(type == "FollowTagTask")
	    {
	        if(msg->task.param_values.size() > 0)
	        {
		        myBid.utility = utiHelp->calculate(this, new FollowTagTask(msg->task.id, msg->task.priority, std::stoi(msg->task.param_values[0])));
	        }
	        else
	        {
		        myBid.utility = utiHelp->calculate(this, new FollowTagTask(msg->task.id, msg->task.priority));
	        }
	    }
	    else if(type == "GoToTask")
	    {
	        myBid.utility = utiHelp->calculate(this, new GoToTask(msg->task));
	    }
	    else if(type == "SearchTask")
	    {
	        myBid.utility = utiHelp->calculate(this, new SearchTask(msg->task));
	    }
        else
        {
	        ROS_ERROR("unrecognized task type %s", type.c_str());
            myBid.utility = 0;
        }
    
        ROS_INFO("my utility is %f, publishing", myBid.utility);
        bidPub.publish(myBid);
    }
    else
    {
        ROS_INFO("already have a task, abstaining from auction");
        myBid.utility = 0;
    }
    
    return myBid.utility;
}

double Thor::bid(const hsmrs_framework::TaskMsg::ConstPtr& msg)
{
    hsmrs_framework::BidMsg myBid = hsmrs_framework::BidMsg();
    myBid.task = *msg;
    myBid.name = getName();
    std::string type = msg->type;
    
    if(p_currentTask == NULL)
    {
        ROS_INFO("calculating utility...");
        if(type == "MyTask")
        {
            myBid.utility = utiHelp->calculate(this, new MyTask(0, 1));
        }
        else if(type == "FollowTagTask")
	    {
	        if(msg->param_values.size() > 0)
	        {
		        myBid.utility = utiHelp->calculate(this, new FollowTagTask(msg->id, msg->priority, std::stoi(msg->param_values[0])));
	        }
	        else
	        {
		        myBid.utility = utiHelp->calculate(this, new FollowTagTask(msg->id, msg->priority));
	        }
	    }
	    else if(type == "GoToTask")
	    {
	        myBid.utility = utiHelp->calculate(this, new GoToTask(msg));
	    }
	    else if(type == "SearchTask")
	    {
	        myBid.utility = utiHelp->calculate(this, new SearchTask(msg));
	    }
        else
        {
	        ROS_ERROR("unrecognized task type %s", type.c_str());
            myBid.utility = 0;
        }
        
        ROS_INFO("my utility is %f, publishing", myBid.utility);
        
        bidPub.publish(myBid);
    }
    else
    {
        ROS_INFO("already have a task, abstaining from auction");
        myBid.utility = 0;
    }
    
    return myBid.utility;
}

void Thor::handleProgress(const std_msgs::String::ConstPtr& msg)
{
    std::string progress = msg->data;
    ROS_INFO("got progress %s", msg->data.c_str());
    if(progress == "complete")
    {
        cancelTask();
    }
}

int main(int argc, char **argv) {
    std::string name = "thor";
    double speed = 1;
    
    if(argc > 1)
    {
        name = std::string(argv[1]);
    }
    if(argc > 2)
    {
        speed = std::stod(std::string(argv[2]));
    }
    
	ros::init(argc, argv, name);

	Thor* robot = new Thor(name, speed);
	return 0;
}

