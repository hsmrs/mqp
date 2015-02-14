#include "hermes/hermes_node.h"

	/**
	 * Begins the Robot's execution of its current Task.
	 */
	void Hermes::executeTask() {
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

	void Hermes::pauseTask(){
		p_currentBehavior->pause();
	}

	void Hermes::resumeTask(){
		p_currentBehavior->resume();	
	}

	void Hermes::doGoToTask(double x, double y){
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
	void Hermes::requestTaskForQueue(Task* task) {

	}


	/**
	 * Send a help request to the Human supervisor.
	 */
	void Hermes::callForHelp() {
		std_msgs::String msg;
		msg.data = "true";
		help_pub.publish(msg);
	}

	void Hermes::handleTeleop(){

	}

	void Hermes::registerWithGUI() {
		//name;requestTopic;logTopic;imageTopic;poseTopic;statusTopic;helpTopic;teleOpTopic

		std_msgs::String msg;
		std::stringstream ss;

		ss << NAME << ";" << REQUEST_TOPIC << ";" << LOG_TOPIC << ";" << IMAGE_TOPIC << ";"
				<< POSE_TOPIC << ";" << STATUS_TOPIC << ";" << HELP_TOPIC << ";" 
				<< TELE_OP_TOPIC;
		msg.data = ss.str();
		registration_pub.publish(msg);
	}

	void Hermes::sendMessage(std::string message){
		std_msgs::String msg;
		msg.data = message;
		log_pub.publish(msg);
	}

	void Hermes::requestCallback(const std_msgs::String::ConstPtr& msg)
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

	void Hermes::teleOpCallback(const geometry_msgs::Twist::ConstPtr& msg){
		double linearVel = msg->linear.x * linearSpeed;
		double angularVel = msg->linear.y * angularSpeed;

		geometry_msgs::Twist velMsg;
		velMsg.linear.x = linearVel;
		velMsg.angular.z = angularVel;

		vel_pub.publish(velMsg);
	}

	void Hermes::newTaskCallback(const std_msgs::String::ConstPtr& msg){
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

	void Hermes::updatedTaskCallback(const std_msgs::String::ConstPtr& msg){

	}

	void Hermes::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

	}

	Hermes::Hermes() : NAME("Hermes"), REGISTRATION_TOPIC("hsmrs/robot_registration"), IMAGE_TOPIC("hermes/camera/rgb/image_mono"), 
			LOG_TOPIC("hermes/log_messages"), STATUS_TOPIC("hermes/status"), HELP_TOPIC("hermes/help"), POSE_TOPIC("hermes/pose"),
			REQUEST_TOPIC("hermes/requests"), TELE_OP_TOPIC("hermes/tele_op"), VEL_TOPIC("hermes/cmd_vel_mux/input/teleop"), NEW_TASK_TOPIC("/hsmrs/new_task"), 
			UPDATED_TASK_TOPIC("/hsmrs/updated_task_topic"), LASER_TOPIC("hermes/scan")
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

		new_task_sub = n.subscribe(NEW_TASK_TOPIC, 1000, &Hermes::newTaskCallback, this);
		updated_task_sub = n.subscribe(UPDATED_TASK_TOPIC, 1000, &Hermes::updatedTaskCallback, this);
		request_sub = n.subscribe(REQUEST_TOPIC, 1000, &Hermes::requestCallback, this);
		teleOp_sub = n.subscribe(TELE_OP_TOPIC, 1000, &Hermes::teleOpCallback, this);

		//Turtlebot publishers and subscribers
		vel_pub = n.advertise<geometry_msgs::Twist>(VEL_TOPIC, 100);
		//laser_sub = n.subscribe(LASER_TOPIC, 1000, &Hermes::laserCallback, this);

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

	std::string Hermes::getName(){
		return NAME;
	}


	/**
	 * Returns the value of the specified attribute from this Robot's AgentState.
	 * @param attr The name of the attribute to get
	 * @return The value of the attribute
	 */
	double Hermes::getAttribute(std::string attr) {

	}

	/**
	 * Returns this Robot's utility for the specified Task.
	 * @param task A pointer to the task for which to get a utility.
	 * @return This Robot's utility for the given Task.
	 */
	double Hermes::getUtility(Task *task) {

	}

	/**
	 * Returns this Robot's AgentState.
	 * @return The AgentState representing the state of this Robot.
	 */
	AgentState* Hermes::getState() {

	}

	/**
	 * Checks if this Robot has the given attribute.
	 * @param attr The name of the target attribute
	 * @return True if the robot has the named attribute.
	 */
	bool Hermes::hasAttribute(std::string attr) {

	}

	/**
	 * Sets this Robot's currently active Task.
	 * @param A pointer to the Task to be set
	 */
	void Hermes::setTask(Task* task) {
		p_currentTask = task;
	}

	/**
	 * Handles the auctioning of Tasks by sending and receiving bids.
	 */
	void Hermes::handleBids(const hsmrs_framework::BidMsg::ConstPtr& msg)
    {
        ROS_INFO("got bid!\n");
    }
	
	/**
	 * Makes this Robot bid on the given task
	 */
    double Hermes::bid(const hsmrs_framework::BidMsg::ConstPtr& msg)
    {
        return 0.0;
    }


	/**
	 * Verifies that an Agent claiming a Task has the highest utility for it. If not, informs the Agent of the Task's proper owner.
	 */
	void Hermes::verifyTaskClaim() {

	}

	/**
	 * Stops execution of the current Task and requests that
	 * the Task be returned to the TaskList.
	 */
	void Hermes::cancelTask() {

	}

	/**
	 * Asks the Agent to claim a task pointed to by \p task.
	 * @param task A pointer to the task object to be claimed.
	 */
	void Hermes::claimTask(Task* task) {
		ROS_INFO("Claiming task!");
		p_currentTask = task;
		executeTask();
	}

	std::string Hermes::getStatus(){
		return status;
	}

	void Hermes::setStatus(std::string newStatus){
		status = newStatus;
		std_msgs::String msg;
		msg.data = status;
		status_pub.publish(msg);
	}

int main(int argc, char **argv) {
	ros::init(argc, argv, "hermes");

	Hermes* robot = new Hermes();
	return 0;
}
