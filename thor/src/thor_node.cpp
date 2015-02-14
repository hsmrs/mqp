#include "thor/thor_node.h"

/**
 * Begins the Robot's execution of its current Task.
 */
void Thor::executeTask() {
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
	p_currentBehavior = behavior;
	behavior->execute();
	setStatus("Busy");
}

void Thor::pauseTask(){
	if (p_currentBehavior != NULL) p_currentBehavior->pause();
}

void Thor::resumeTask(){
	if (p_currentBehavior != NULL) p_currentBehavior->resume();	
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

	ss << NAME << ";" << REQUEST_TOPIC << ";" << LOG_TOPIC << ";" << IMAGE_TOPIC << ";"
			<< POSE_TOPIC << ";" << STATUS_TOPIC << ";" << HELP_TOPIC << ";" 
			<< TELE_OP_TOPIC;
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

void Thor::newTaskCallback(const std_msgs::String::ConstPtr& msg){
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

void Thor::updatedTaskCallback(const std_msgs::String::ConstPtr& msg){

}

void Thor::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

}

void Thor::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
	//Figure out which tag we are looking at.
	int tagID = msg->markers[0].id;
	std::stringstream ss;
	ss << "hsmrs/marker_" << tagID;
	std::string markerFrameID = ss.str();

	//Manually create transform from robot to tag
	tf::Transform transform;

	double x, y;
	x = msg->markers[0].pose.pose.position.x;
	y = msg->markers[0].pose.pose.position.y;

	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->markers[0].pose.pose.orientation, q);

	transform.setOrigin( tf::Vector3(x, y, 0.0) );
	transform.setRotation(q);

	//Broadcast transform
	tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), markerFrameID, "thor"));
	
	//Listen for transform from map to robot
	tf::TransformListener listener;
	tf::StampedTransform mapTransform;
	try{
		listener.lookupTransform("map", "thor",
			ros::Time(0), mapTransform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		return;
	}

	//Unpack position and orientation
	double map_x, map_y;
	map_x = mapTransform.getOrigin().x();
	map_y = mapTransform.getOrigin().y();

	geometry_msgs::Quaternion quat;
	tf::quaternionTFToMsg(mapTransform.getRotation(), quat);

	//Publish pose as geometry message
	geometry_msgs::Pose pose;
	pose.position.x = map_x;
	pose.position.y = map_y;
	pose.orientation = quat;

	pose_pub.publish(pose);
}

Thor::Thor() : NAME("Thor"), REGISTRATION_TOPIC("hsmrs/robot_registration"), IMAGE_TOPIC("thor/camera/rgb/image_mono"), 
		LOG_TOPIC("thor/log_messages"), STATUS_TOPIC("thor/status"), HELP_TOPIC("thor/help"), POSE_TOPIC("thor/pose"),
		REQUEST_TOPIC("thor/requests"), TELE_OP_TOPIC("thor/tele_op"), VEL_TOPIC("thor/cmd_vel_mux/input/teleop"),
		BUMPER_TOPIC("/thor/mobile_base/events/bumper"), NEW_TASK_TOPIC("/hsmrs/new_task"), 
		UPDATED_TASK_TOPIC("/hsmrs/updated_task_topic"), LASER_TOPIC("thor/scan")
		{

	taskList = new MyTaskList();

	linearSpeed = 0.3;
	angularSpeed = 0.8;

	ros::AsyncSpinner spinner(1);
	spinner.start();

	//GUI Publishers and subscribers
	registration_pub = n.advertise<std_msgs::String>(REGISTRATION_TOPIC, 100);
	log_pub = n.advertise<std_msgs::String>(LOG_TOPIC, 100);
	status_pub = n.advertise<std_msgs::String>(STATUS_TOPIC, 100);
	help_pub = n.advertise<std_msgs::String>(HELP_TOPIC, 100);
	pose_pub = n.advertise<geometry_msgs::PoseStamped>(POSE_TOPIC, 100);

	new_task_sub = n.subscribe(NEW_TASK_TOPIC, 1000, &Thor::newTaskCallback, this);
	updated_task_sub = n.subscribe(UPDATED_TASK_TOPIC, 1000, &Thor::updatedTaskCallback, this);
	request_sub = n.subscribe(REQUEST_TOPIC, 1000, &Thor::requestCallback, this);
	teleOp_sub = n.subscribe(TELE_OP_TOPIC, 1000, &Thor::teleOpCallback, this);

	//Turtlebot publishers and subscribers
	vel_pub = n.advertise<geometry_msgs::Twist>(VEL_TOPIC, 100);
	bumper_sub = n.subscribe(BUMPER_TOPIC, 1000, &Thor::bumperCallback, this);
	tag_sub = n.subscribe("thor/ar_pose_markers", 1000, &Thor::tagCallback, this);
	//laser_sub = n.subscribe(LASER_TOPIC, 1000, &Thor::laserCallback, this);

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

}

/**
 * Checks if this Robot has the given attribute.
 * @param attr The name of the target attribute
 * @return True if the robot has the named attribute.
 */
bool Thor::hasAttribute(std::string attr) {

}

/**
 * Sets this Robot's currently active Task.
 * @param A pointer to the Task to be set
 */
void Thor::setTask(Task* task) {
	p_currentTask = task;
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

}

/**
 * Asks the Agent to claim a task pointed to by \p task.
 * @param task A pointer to the task object to be claimed.
 */
void Thor::claimTask(Task* task) {
	ROS_INFO("Claiming task!");
	p_currentTask = task;
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

/**
 * Handles the auctioning of Tasks by sending and receiving bids.
 */
void Thor::handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg)
{
    ROS_INFO("got bid!\n");
}

/**
 * Makes this Robot bid on the given task
 */
double Thor::bid(const hsmrs_framework::BidMsg::ConstPtr& msg)
{
    return 0.0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "thor");

	Thor* robot = new Thor();
	return 0;
}
