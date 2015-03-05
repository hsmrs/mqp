#include "thor/SearchBehavior.h"

int counter = 0;
std::atomic_uint protected_progress;
std::atomic_bool protected_isExecuting;

ros::Publisher g_goalPub;
geometry_msgs::PointStamped g_goalMsg;
ros::Publisher g_cancelPub;
std_msgs::String g_cancelMsg;
ros::Publisher g_cmdVelPub;

Robot* g_parent;

double g_maxLinearVelocity, g_maxAngularVelocity;
int g_tagID;

bool g_isFound = false;

std::vector<geometry_msgs::PointStamped> g_goals;

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
    //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	if (!protected_isExecuting.load() || 
		msg->markers.size() == 0) return;

	for (int i = 0; i < msg->markers.size(); ++i){
		if (msg->markers[i].id == g_tagID){
			if (!g_isFound) g_cancelPub.publish(g_cancelMsg);
			g_isFound = true;

			double linK = 0.4;
			double angK = 2;	
			double x = msg->markers[i].pose.pose.position.x;
			double y = msg->markers[i].pose.pose.position.y;
			double linVel = std::max(std::min(g_maxLinearVelocity, linK * x), 0.0);
			double angVel = std::max(std::min(g_maxAngularVelocity, angK * y), -1*g_maxAngularVelocity);

			if (x <= 1.0){
				linVel = 0;
				g_parent->sendMessage("I found it!");
				ROS_INFO("I found it!");//" %s", info.c_str());
				protected_isExecuting.store(false);
				//std::unique_lock<std::recursive_mutex> progressLock(progressMutex);
				protected_progress.store(100);
				//std_msgs::String progMsg = std_msgs::String();
				//progMsg.data = "complete";
				//progressPub.publish(progMsg);
			}

			geometry_msgs::Twist velMsg;
			velMsg.linear.x = linVel;
			velMsg.angular.z = angVel;

			g_cmdVelPub.publish(velMsg);

			return;
		}
	}
	if (g_isFound) g_goalPub.publish(g_goalMsg);
	g_isFound = false;
	//isExecutingLock.unlock();
	boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

void SearchBehavior::progressCallback(const std_msgs::String::ConstPtr& msg){
    //boost::mutex::scoped_lock isExecutingLock(isExecutingMutex);
    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	if (!protected_isExecuting.load()) return;
	
	std::string msgProgress = msg->data;
	if (msgProgress == "complete" && protected_isExecuting.load()){
		if (!g_isFound){
			ROS_INFO("Getting next target!");
			ROS_INFO("(%f, %f)", g_goals[counter].point.x, g_goals[counter].point.y);
			if (counter < g_goals.size()) {
				g_goalMsg = goals[counter++];
				ROS_INFO("publishing goal");// %s", info.c_str());
				g_goalPub.publish(g_goalMsg);
			}
			else {
				g_parent->sendMessage("I could not find it.");
				ROS_INFO("I could not find it.");//" %s", info.c_str());
				protected_isExecuting.store(false);
				//boost::mutex::scoped_lock progressLock(progressMutex);
				//std::unique_lock<std::recursive_mutex> progressLock(progressMutex);
				protected_progress.store(100);
				//progressLock.unlock();
				//progressPub.publish(*msg);
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
	//this->parent = parent;
	g_parent = parent;
	isExecuting = false;
	isFound = false;

	// this->maxLinearVelocity = maxLinearVelocity;
	// this->maxAngularVelocity = maxAngularVelocity;
	g_maxAngularVelocity = maxAngularVelocity;
	g_maxLinearVelocity = maxLinearVelocity;
	//this->tagID = tagID;
	g_tagID = tagID;
	this->boundaryVertices = boundaryVertices;
	this->resolution = resolution;
	counter = 0;
	createGoals();
	g_goalMsg = g_goals[counter++];
	//pop_front<geometry_msgs::PointStamped>(goals, goalMsg);
    info = "robot: " + parent->getName();

    protected_progress.store(0);
    protected_isExecuting.store(false);

	g_goalPub = nh.advertise<geometry_msgs::PointStamped>("navigation/goal", 1000, true);
	g_cancelMsg.data = "cancel";
	//progressPub = nh.advertise<std_msgs::String>("progress", 1000);
	g_cancelPub = nh.advertise<std_msgs::String>("navigation/cancel", 1000);
	g_cmdVelPub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 1000);

	progressSub = nh.subscribe("navigation/progress", 1000, &SearchBehavior::progressCallback, this);
	markerSub = nh.subscribe(MARKER_TOPIC, 1, &SearchBehavior::tagCallback, this);
}

SearchBehavior::~SearchBehavior(){
	g_goalPub.shutdown();
	progressPub.shutdown();
	g_cancelPub.shutdown();
	g_cmdVelPub.shutdown();
	progressSub.shutdown();
	markerSub.shutdown();
	nh.shutdown();
}

void SearchBehavior::execute(){
    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	ROS_INFO("Executing search behavior! %s", info.c_str());
	protected_isExecuting.store(true);
	g_goalPub.publish(goalMsg);
}

void SearchBehavior::resume(){
    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	protected_isExecuting.store(true);
	g_goalPub.publish(goalMsg);
}

void SearchBehavior::pause(){
    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	protected_isExecuting.store(false);
	g_cancelPub.publish(cancelMsg);
}

void SearchBehavior::stop(){
    //std::unique_lock<std::recursive_mutex> isExecutingLock(isExecutingMutex);
	protected_isExecuting.store(false);
	g_cancelPub.publish(cancelMsg);
}

std::string SearchBehavior::checkProgress(){
	//boost::mutex::scoped_lock progressLock(progressMutex);
	//std::unique_lock<std::recursive_mutex> progressLock(progressMutex);
	return (protected_progress.load() == 100 ? "complete" : "");
}

void SearchBehavior::createGoals(){
	ROS_INFO("Creating goals!");
	//ROS_INFO("Vertices = (%f, %f) (%f, %f) (%f, %f) (%f, %f)", boundaryVertices[0].point.x, boundaryVertices[0].point.y,boundaryVertices[1].point.x,boundaryVertices[1].point.y,boundaryVertices[2].point.x,boundaryVertices[2].point.y,boundaryVertices[3].point.x,boundaryVertices[3].point.y);
	geometry_msgs::PointStamped vertex1 = boundaryVertices[0];
	geometry_msgs::PointStamped vertex2 = boundaryVertices[1];
	geometry_msgs::PointStamped vertex3 = boundaryVertices[2];
	geometry_msgs::PointStamped vertex4 = boundaryVertices[3];

	int direction = -1;
	double curY = vertex2.point.y;
	//ROS_INFO("Begin pushing goals");
	g_goals.push_back(vertex1);
	g_goals.push_back(vertex2);
	while(curY + resolution < vertex4.point.y){
		curY += resolution;
		ROS_INFO("Current Y: %f", curY);
		geometry_msgs::PointStamped temp1;
		geometry_msgs::PointStamped temp2;

		temp1.point.x = vertex2.point.x;
		temp1.point.y = curY;
		temp2.point.x = vertex1.point.x;
		temp2.point.y = curY;
		
		//ROS_INFO("Adding points: (%f, %f) and (%f, %f)", temp1.point.x, temp1.point.y, temp2.point.x, temp2.point.y);
		
		if (direction == -1){
			//ROS_INFO("Right to left");
			g_goals.push_back(temp1);
			g_goals.push_back(temp2);
		}
		else{
			//ROS_INFO("Left to right");
			g_goals.push_back(temp2);
			g_goals.push_back(temp1);
		}
		//ROS_INFO("Goals pushed");
		direction *= -1;
	}
	//ROS_INFO("Adding last vertices");
	if (direction == -1)
	{
		g_goals.push_back(vertex4);
		g_goals.push_back(vertex3);
	}
	else {
		g_goals.push_back(vertex3);
		g_goals.push_back(vertex4);
	}
	ROS_INFO("Goals created!");
	for (int i = 0; i < g_goals.size(); i++){
		geometry_msgs::PointStamped msg = g_goals[i];	
		ROS_INFO("(%f, %f)", msg.point.x, msg.point.y);
	}
}
