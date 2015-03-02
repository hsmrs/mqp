#include "thor/SearchBehavior.h"

int counter = 0;

//Quick and dirty utility function
template<typename T>
bool pop_front(std::vector<T>& vec, T& removedItem)
{
    if (!vec.empty()){
    	removedItem = vec[0];
    	vec.erase(vec.begin());
    	return true;
	}
	return false;
}


void SearchBehavior::tagCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	if (!isExecuting || 
		msg->markers.size() == 0) return;

	for (int i = 0; i < msg->markers.size(); ++i){
		if (msg->markers[i].id == tagID){
			if (!isFound) cancelPub.publish(cancelMsg);
			isFound = true;

			double linK = 0.4;
			double angK = 2;	
			double x = msg->markers[i].pose.pose.position.x;
			double y = msg->markers[i].pose.pose.position.y;
			double linVel = std::max(std::min(maxLinearVelocity, linK * x), 0.0);
			double angVel = std::max(std::min(maxAngularVelocity, angK * y), -1*maxAngularVelocity);

			if (x <= 1.0){
				linVel = 0;
				parent->sendMessage("I found it!");
				ROS_INFO("I found it!");
				isExecuting = false;
				std_msgs::String progMsg = std_msgs::String();
				progMsg.data = "complete";
				progressPub.publish(progMsg);
			}

			geometry_msgs::Twist velMsg;
			velMsg.linear.x = linVel;
			velMsg.angular.z = angVel;

			cmdVelPub.publish(velMsg);

			return;
		}
	}
	if (isFound) goalPub.publish(goalMsg);
	isFound = false;
}

void SearchBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	if (!isExecuting) return;
	
	std::string progress = msg->data;
	if (progress == "complete" && isExecuting){
		if (!isFound){
			ROS_INFO("Getting next target!");
			ROS_INFO("(%f, %f)", goals[counter].point.x, goals[counter].point.y);
			if (counter < goals.size()) {
				goalMsg = goals[counter++];
				goalPub.publish(goalMsg);
			}
			else {
				parent->sendMessage("I could not find it.");
				ROS_INFO("I could not find it.");
				isExecuting = false;
				progressPub.publish(*msg);
			}
			//bool success = pop_front<geometry_msgs::PointStamped>(boundaryVertices, goalMsg);
			//ROS_INFO("(%f, %f)", goalMsg.point.x, goalMsg.point.y);
			//if (success) goalPub.publish(goalMsg);
			//else {
			//	parent->sendMessage("I could not find it.");
			//	stop();
			//}
		}
	}
}


SearchBehavior::SearchBehavior(Robot* parent, double maxLinearVelocity, double maxAngularVelocity, double resolution, 
	int tagID, std::vector<geometry_msgs::PointStamped> boundaryVertices, ros::NodeHandle n, std::string cmdVelTopic) : 
MARKER_TOPIC("ar_pose_marker")
{
	this->parent = parent;
	isExecuting = false;
	isFound = false;

	this->maxLinearVelocity = maxLinearVelocity;
	this->maxAngularVelocity = maxAngularVelocity;
	this->tagID = tagID;
	this->boundaryVertices = boundaryVertices;
	this->resolution = resolution;
	counter = 0;
	createGoals();
	goalMsg = goals[counter++];
	//pop_front<geometry_msgs::PointStamped>(goals, goalMsg);


	goalPub = n.advertise<geometry_msgs::PointStamped>("navigation/goal", 1000, true);
	cancelMsg.data = "cancel";
	progressPub = n.advertise<std_msgs::String>("progress", 1000);
	cancelPub = n.advertise<std_msgs::String>("navigation/cancel", 1000);
	cmdVelPub = n.advertise<geometry_msgs::Twist>(cmdVelTopic, 1000);

	progressSub = n.subscribe("navigation/progress", 1000, &SearchBehavior::progressCallback, this);
	markerSub = n.subscribe(MARKER_TOPIC, 1000, &SearchBehavior::tagCallback, this);
}

void SearchBehavior::execute(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	ROS_INFO("Executing search behavior!");
	isExecuting = true;
	goalPub.publish(goalMsg);
}

void SearchBehavior::resume(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = true;
	goalPub.publish(goalMsg);
}

void SearchBehavior::pause(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = false;
	cancelPub.publish(cancelMsg);
}

void SearchBehavior::stop(){
    boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
	isExecuting = false;
	cancelPub.publish(cancelMsg);
}


void SearchBehavior::createGoals(){
	ROS_INFO("Creating goals!");
	ROS_INFO("Vertices = (%f, %f) (%f, %f) (%f, %f) (%f, %f)", boundaryVertices[0].point.x, boundaryVertices[0].point.y,boundaryVertices[1].point.x,boundaryVertices[1].point.y,boundaryVertices[2].point.x,boundaryVertices[2].point.y,boundaryVertices[3].point.x,boundaryVertices[3].point.y);
	geometry_msgs::PointStamped vertex1 = boundaryVertices[0];
	geometry_msgs::PointStamped vertex2 = boundaryVertices[1];
	geometry_msgs::PointStamped vertex3 = boundaryVertices[2];
	geometry_msgs::PointStamped vertex4 = boundaryVertices[3];

	int direction = -1;
	double curY = vertex2.point.y;
	ROS_INFO("Begin pushing goals");
	goals.push_back(vertex1);
	goals.push_back(vertex2);
	while(curY + resolution < vertex4.point.y){
		curY += resolution;
		ROS_INFO("Current Y: %f", curY);
		geometry_msgs::PointStamped temp1;
		geometry_msgs::PointStamped temp2;

		temp1.point.x = vertex2.point.x;
		temp1.point.y = curY;
		temp2.point.x = vertex1.point.x;
		temp2.point.y = curY;
		
		ROS_INFO("Adding points: (%f, %f) and (%f, %f)", temp1.point.x, temp1.point.y, temp2.point.x, temp2.point.y);
		
		if (direction == -1){
			ROS_INFO("Right to left");
			goals.push_back(temp1);
			goals.push_back(temp2);
		}
		else{
			ROS_INFO("Left to right");
			goals.push_back(temp2);
			goals.push_back(temp1);
		}
		ROS_INFO("Goals pushed");
		direction *= -1;
	}
	ROS_INFO("Adding last vertices");
	if (direction == -1)
	{
		goals.push_back(vertex4);
		goals.push_back(vertex3);
	}
	else {
		goals.push_back(vertex3);
		goals.push_back(vertex4);
	}
	ROS_INFO("Goals created!");
	for (int i = 0; i < goals.size(); i++){
		geometry_msgs::PointStamped msg = goals[i];	
		ROS_INFO("(%f, %f)", msg.point.x, msg.point.y);
	}
}
