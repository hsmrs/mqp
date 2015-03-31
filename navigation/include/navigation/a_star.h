#ifndef _ASTAR_H_
#define _ASTAR_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

class AStar {
private:
	ros::Subscriber goalSub;
	ros::Subscriber mapSub;
	ros::Subscriber poseSub;

	ros::Publisher pathPub;

	int mapGridHeight;
	int mapGridWidth;
	int[][] mapData;
	int[][] heuristic;

public:

	AStar();

	void goalCallback(const geometry_msgs::Point::ConstPtr& goal_msg);

	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg);

	void poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg);

	void createHeuristic(geometry_msgs::Point goalPoint);

	void createPath(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint);



};

#endif