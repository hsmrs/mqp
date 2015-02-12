#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

#include "AStarNode.h"
#include <math.h>
#include <queue>


class AStar {
private:
	ros::Subscriber goalSub;
	ros::Subscriber mapSub;
	ros::Subscriber poseSub;

	ros::Publisher pathPub;

	geometry_msgs::Pose currentPose;

	int mapGridHeight;
	int mapGridWidth;
	int** mapData;

public:

	AStar();

	void goalCallback(const geometry_msgs::Point::ConstPtr& goal_msg);

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

	void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg);

	void createHeuristic(geometry_msgs::Point goalPoint, int** target);

	void createPath(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint);

	void reconstructPath(AStarNode current, std::map<AStarNode, AStarNode> parents);

};

#endif