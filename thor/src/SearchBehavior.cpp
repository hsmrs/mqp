#include "thor/SearchBehavior.h"

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
				stop();
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
	std::string progress = msg->data;
	if (progress == "complete"){
		if (!isFound){
			bool success = pop_front<geometry_msgs::PointStamped>(boundaryVertices, goalMsg);
			if (success) goalPub.publish(goalMsg);
			else {
				parent->sendMessage("I could not find it.");
				stop();
			}
		}
	}
}


SearchBehavior::SearchBehavior(Robot* parent, double maxLinearVelocity, double maxAngularVelocity, double resolution, 
	int tagID, std::vector<geometry_msgs::PointStamped> boundaryVertices, ros::NodeHandle n, std::string cmdVelTopic) : 
MARKER_TOPIC("/thor/ar_pose_marker")
{
	this->parent = parent;
	isExecuting = false;
	isFound = false;

	this->maxLinearVelocity = maxLinearVelocity;
	this->maxAngularVelocity = maxAngularVelocity;
	this->tagID = tagID;
	this->boundaryVertices = boundaryVertices;

	createGoals();
	pop_front<geometry_msgs::PointStamped>(goals, goalMsg);


	goalPub = n.advertise<geometry_msgs::PointStamped>("thor/navigation/goal", 1000, true);
	cancelMsg.data = "cancel";
	cancelPub = n.advertise<std_msgs::String>("thor/navigation/cancel", 1000);
	cmdVelPub = n.advertise<geometry_msgs::Twist>(cmdVelTopic, 1000);

	progressSub = n.subscribe("thor/navigation/progress", 1000, &SearchBehavior::progressCallback, this);
	markerSub = n.subscribe(MARKER_TOPIC, 1000, &SearchBehavior::tagCallback, this);
}

void SearchBehavior::execute(){
	isExecuting = true;
	goalPub.publish(goalMsg);
}

void SearchBehavior::resume(){
	isExecuting = true;
	goalPub.publish(goalMsg);
}

void SearchBehavior::pause(){
	isExecuting = false;
	cancelPub.publish(cancelMsg);
}

void SearchBehavior::stop(){
	isExecuting = false;
	cancelPub.publish(cancelMsg);
}


void SearchBehavior::createGoals(){
	geometry_msgs::PointStamped vertex1 = boundaryVertices[0];
	geometry_msgs::PointStamped vertex2 = boundaryVertices[1];
	geometry_msgs::PointStamped vertex3 = boundaryVertices[2];
	geometry_msgs::PointStamped vertex4 = boundaryVertices[3];

	int direction = -1;
	double curY = vertex2.point.y;

	goals.push_back(vertex1);
	goals.push_back(vertex2);
	while(curY + resolution <= vertex4.point.y){
		geometry_msgs::PointStamped temp1;
		geometry_msgs::PointStamped temp2;

		temp1.point.x = vertex2.point.x;
		temp1.point.y = curY;
		temp2.point.x = vertex1.point.x;
		temp2.point.y = curY;
		
		if (direction == -1){
			goals.push_back(temp1);
			goals.push_back(temp2);
		}
		else{
			goals.push_back(temp2);
			goals.push_back(temp1);
		}

		curY += resolution;
		direction *= -1;
	}

	if (direction == -1)
	{
		goals.push_back(vertex4);
		goals.push_back(vertex3);
	}
	else {
		goals.push_back(vertex3);
		goals.push_back(vertex4);
	}
}