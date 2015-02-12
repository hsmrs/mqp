#include "navigation/AStar.h"

	AStar::AStar(){
		ros::NodeHandle nh;
		ros::AsyncSpinner spinner(1);
		spinner.start();
		
		std::string poseTopic;
		nh.param<std::string>("pose_topic", poseTopic, "pose");
		poseSub = nh.subscribe(poseTopic, 1000, &AStar::poseCallback, this);

		goalSub = nh.subscribe("navigation/goal", 1000, &AStar::goalCallback, this);
		mapSub = nh.subscribe("map", 1000, &AStar::mapCallback, this);

		pathPub = nh.advertise<nav_msgs::Path>("navigation/path", 1000);

		while (ros::ok()){}
	}

	void AStar::goalCallback(const geometry_msgs::Point::ConstPtr& goal_msg){

	}

	void AStar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
		mapGridHeight = map_msg->info.height;
		mapGridHeight = map_msg->info.width;

		int i = 0;
		for (int y = 0; y < mapGridHeight; ++y){
			for (int x = 0; x < mapGridWidth; ++x){
				mapData[y][x] = map_msg->data[i++];
			}
		}
	}

	void AStar::poseCallback(const geometry_msgs::Pose::ConstPtr& pose_msg){
		currentPose = *pose_msg;
	}

	void AStar::createHeuristic(geometry_msgs::Point goalPoint, int** target){
		//int heuristic[mapGridHeight][mapGridWidth];
		target = new int*[mapGridHeight];

		int goalX = goalPoint.x;
		int goalY = goalPoint.y;

		for (int y = 0; y < mapGridHeight; ++y){
			target[y] = new int[mapGridWidth];
			for (int x = 0; x < mapGridWidth; ++x){
				target[y][x] = (int) sqrt(pow(x - goalY, 2) + pow(y - goalY, 2));
			}
		}
	}

	void AStar::createPath(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint){
		int** heuristic;
		createHeuristic(goalPoint, heuristic);
		std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode> > frontier;
		bool visited[mapGridHeight][mapGridWidth];
		std::map<AStarNode, AStarNode> parents;

		int x = startPoint.x;
		int y = startPoint.y;
		AStarNode start(x, y, heuristic[y][x]);
		frontier.push(start);

		while (frontier.empty() != true){
			AStarNode current = frontier.top();
			frontier.pop();

			int cur_x = current.getX();
			int cur_y = current.getY();
			int cur_cost = current.getCost();

			if (cur_x == goalPoint.x && cur_y == goalPoint.y){
				reconstructPath(current, parents);
				return; 
			}

			visited[cur_y][cur_x] = true;

			std::vector<AStarNode> neighbors;
			if (cur_x - 1 >= 0){
				if (cur_y - 1 >= 0){
					AStarNode node1(cur_x - 1, cur_y - 1, 0);
					AStarNode node2(cur_x - 1, 0, 0);
					AStarNode node3(0, cur_y - 1, 0);

					neighbors.push_back(node1);
					neighbors.push_back(node2);
					neighbors.push_back(node3);
				} 
				if (cur_y + 1 < mapGridHeight){
					AStarNode node1(cur_x - 1, cur_y + 1, 0);
					AStarNode node2(cur_x - 1, 0, 0);
					AStarNode node3(0, cur_y + 1, 0);

					neighbors.push_back(node1);
					neighbors.push_back(node2);
					neighbors.push_back(node3);
				}
			} 
			
			if (cur_x + 1 < mapGridWidth){
				if (cur_y - 1 >= 0){
					AStarNode node1(cur_x + 1, cur_y - 1, 0);
					AStarNode node2(cur_x + 1, 0, 0);
					AStarNode node3(0, cur_y - 1, 0);

					neighbors.push_back(node1);
					neighbors.push_back(node2);
					neighbors.push_back(node3);
				} 
				if (cur_y + 1 < mapGridHeight){
					AStarNode node1(cur_x + 1, cur_y + 1, 0);
					AStarNode node2(cur_x + 1, 0, 0);
					AStarNode node3(0, cur_y + 1, 0);

					neighbors.push_back(node1);
					neighbors.push_back(node2);
					neighbors.push_back(node3);
				}
			}

			for (int i = 0; i < neighbors.size(); ++i){
				AStarNode neighborNode = neighbors[i];
				if (visited[neighborNode.getY()][neighborNode.getX()]){
					continue;
				}

				int newCost = cur_cost + 1 + heuristic[neighborNode.getY()][neighborNode.getX()];
				neighborNode.setCost(newCost);
				frontier.push(neighborNode);
				parents[neighborNode] = current;	
			}

		}
		ROS_INFO("Failed to find a path!");
	}

	void AStar::reconstructPath(AStarNode current, std::map<AStarNode, AStarNode> parents){
		std::vector<AStarNode> path;

		path.push_back(current);
		while(parents.find(current) != parents.end()){
			current = parents[current];
			path.push_back(current);
		}
		std::reverse(path.begin(), path.end());

		nav_msgs::Path pathMsg;
		for (int i = 0; i < path.size(); ++i){
			pathMsg.poses[i].pose.position.x = path[i].getX();
			pathMsg.poses[i].pose.position.y = path[i].getY();
		}

		pathPub.publish(pathMsg);
	}

int main(int argc, char **argv) {
	ros::init(argc, argv, "astar_planner");

	AStar* planner = new AStar();
	return 0;
}