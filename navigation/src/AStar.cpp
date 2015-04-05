#include "navigation/AStar.h"

	AStar::AStar(){
		ros::NodeHandle nh;
		ros::AsyncSpinner spinner(1);
		spinner.start();
		
		std::string poseTopic;
		std::string mapTopic;
		ros::param::param<std::string>("~pose_topic", poseTopic, "pose");
		ros::param::param<std::string>("~map_topic", mapTopic, "map");

		ROS_INFO("Listenening for pose on: %s", poseTopic.c_str());
		ROS_INFO("Listenening for map on: %s", mapTopic.c_str());

		poseSub = nh.subscribe(poseTopic, 1000, &AStar::poseCallback, this);
		goalSub = nh.subscribe("navigation/goal", 1000, &AStar::goalCallback, this);
		mapSub = nh.subscribe(mapTopic, 1000, &AStar::mapCallback, this);

		pathPub = nh.advertise<nav_msgs::Path>("navigation/path", 1000);

		while (ros::ok()){}
	}

	void AStar::goalCallback(const geometry_msgs::PointStamped::ConstPtr& goal_msg){
		ROS_INFO("Received goal!");
		geometry_msgs::Point startPoint = currentPose.position;
		createPath(startPoint, goal_msg->point);
	}

	void AStar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
		mapGridHeight = map_msg->info.height;
		mapGridWidth = map_msg->info.width;
		mapResolution = map_msg->info.resolution;
		mapData = new int*[mapGridHeight];

		int i = 0;
		for (int y = 0; y < mapGridHeight; ++y){
			mapData[y] = new int[mapGridWidth];
			for (int x = 0; x < mapGridWidth; ++x){
				mapData[y][x] = map_msg->data[i++];
			}
		}

		ROS_INFO("Received map of height %d and width %d", mapGridHeight, mapGridWidth);
	}

	void AStar::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){
		currentPose = pose_msg->pose;
	}

	void AStar::createHeuristic(geometry_msgs::Point goalPoint, int** target){
		//ROS_INFO("Creating heuristic");
		target = new int*[mapGridHeight];

		int goalX = (goalPoint.x)/mapResolution;
		int goalY = (goalPoint.y)/mapResolution;

		for (int y = 0; y < mapGridHeight; ++y){
			target[y] = new int[mapGridWidth];
			for (int x = 0; x < mapGridWidth; ++x){
				target[y][x] = (int) sqrt(pow(x - goalY, 2) + pow(y - goalY, 2));
			}
		}
		//ROS_INFO("Heuristic created!");
	}

	AStarNode current;
	void AStar::createPath(geometry_msgs::Point startPoint, geometry_msgs::Point goalPoint){
		if (mapGridWidth == 0 || mapGridHeight == 0){
			ROS_INFO("No map, cannot find path!");
			return;
		}

		ROS_INFO("Finding path!");
		
		//ROS_INFO("Creating heuristic");
		int** heuristic = new int*[mapGridHeight];

		int goalX = (goalPoint.x)/mapResolution;
		int goalY = (goalPoint.y)/mapResolution;

		for (int y = 0; y < mapGridHeight; ++y){
			heuristic[y] = new int[mapGridWidth];
			for (int x = 0; x < mapGridWidth; ++x){
				heuristic[y][x] = (int) sqrt(pow(x - goalY, 2) + pow(y - goalY, 2));
			}
		}
		//ROS_INFO("Heuristic created!");

		//ROS_INFO("%d x %d", mapGridHeight, mapGridWidth);

		//int** heuristic;
		//createHeuristic(goalPoint, heuristic);

		std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode> > frontier;
		bool visited[mapGridHeight][mapGridWidth];
		std::map<AStarNode, AStarNode> parents;
		bool onFrontier[mapGridHeight][mapGridWidth];

		for (int i =0; i < mapGridHeight; ++i){
			for (int j = 0; j < mapGridWidth; ++j){
				visited[i][j]= false;
			}
		}

		for (int i =0; i < mapGridHeight; ++i){
			for (int j = 0; j < mapGridWidth; ++j){
				onFrontier[i][j]= false;
			}
		}

		int x = (startPoint.x)/mapResolution;
		int y = (startPoint.y)/mapResolution;
		int startCost = heuristic[y][x];
		AStarNode startNode(x, y, startCost);
		
		frontier.push(startNode);
		onFrontier[startNode.getY()][startNode.getX()] = true;

		while (frontier.empty() != true){
			current = frontier.top();
			frontier.pop();
			onFrontier[current.getY()][current.getX()] = false;

			int cur_x = current.getX();
			int cur_y = current.getY();
			int cur_cost = current.getCost();

			if (visited[cur_y][cur_x]) continue;

			//ROS_INFO("At node: (%d, %d)", cur_x, cur_y);

			if (cur_x == (goalPoint.x)/mapResolution && cur_y == (goalPoint.y)/mapResolution){
				//ROS_INFO("Path found! Reconstructing");
				reconstructPath(&current, parents);
				return; 
			}

			visited[cur_y][cur_x] = true;

			std::vector<AStarNode> neighbors;

			for (int i = -1; i < 2; ++i){
				for (int j = -1; j < 2; ++j){
					if (i == 0 && j == 0){
						continue;
					}

					int nx = cur_x + i;
					int ny = cur_y + j;
					if ((nx >= 0) && (nx < mapGridWidth)
						&& (ny >= 0) && (ny < mapGridHeight)){
							AStarNode node(nx, ny, 0);
							neighbors.push_back(node);
					}
				}
			}
			// std::stringstream ss1;
			// for (AStarNode node : neighbors){
   //  			ss1 << "(" << node.getX() << ', ' << node.getY() << "), ";
			// }
			// ROS_INFO("Neighbors");
			// ROS_INFO(ss1.str().c_str());

			for (int i = 0; i < neighbors.size(); ++i){
				AStarNode neighborNode = neighbors[i];
				//ROS_INFO("On frontier? %s", onFrontier[neighborNode.getY()][neighborNode.getX()] ? "true" : "false");
				if (visited[neighborNode.getY()][neighborNode.getX()]){
					continue;
				}

				if (onFrontier[neighborNode.getY()][neighborNode.getX()]){
					continue;
				}

				onFrontier[neighborNode.getY()][neighborNode.getX()] = true;
				int newCost = (cur_cost - heuristic[cur_y][cur_x]) 
					+ 1 + heuristic[neighborNode.getY()][neighborNode.getX()];
				neighborNode.setCost(newCost);
				frontier.push(neighborNode);
				neighborNode.setParent(current);
				AStarNode parent = *(neighborNode.getParent());
				//parents[neighborNode] = current;
				//AStarNode parent = parents[neighborNode];
				//ROS_INFO("(%d, %d)", parent.getX(), parent.getY());
			}

			 std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode> > print_queue = frontier;
			 	std::stringstream ss;
			 	while(!print_queue.empty()){
			 		AStarNode print_node = print_queue.top();
			 		print_queue.pop();
			 		ss << "(" << print_node.getX() << ", " << print_node.getY() << "), ";
			 	}	
			 	//ROS_INFO("Frontier");
			 	//ROS_INFO(ss.str().c_str());

		}
		ROS_INFO("Failed to find a path!");
	}

	void AStar::reconstructPath(AStarNode* current, std::map<AStarNode, AStarNode> parents){
		//ROS_INFO("Reconstructiong!");
		std::vector<AStarNode> path;
		//AStarNode parent = parents[current];
		//ROS_INFO("(%d, %d)", parent.getX(), parent.getY());
		path.push_back(*current);
		
		// while(parents.find(current) != parents.end()){
		// 	current = parents[current];
		// 	ROS_INFO("(%d, %d)", current.getX(), current.getY());
		// 	path.push_back(current);
		// }

		while(current->hasParent){//current->getParent() != NULL){
			current = current->getParent();
			ROS_INFO("(%d, %d)", current->getX(), current->getY());
			path.push_back(*current);
		}

		//ROS_INFO("Reversing!");
		std::reverse(path.begin(), path.end());

		//ROS_INFO("Building message!");
		nav_msgs::Path pathMsg;
		pathMsg.poses.resize(path.size());
		for (int i = 0; i < path.size(); ++i){
			pathMsg.poses[i].pose.position.x = (path[i].getX())*mapResolution;
			pathMsg.poses[i].pose.position.y = (path[i].getY())*mapResolution;
		}
		for (int i = 0; i < path.size(); ++i){
			ROS_INFO("(%f, %f)", path[i].getX()*mapResolution, path[i].getY()*mapResolution);
		}
		pathMsg.header.frame_id = "map";
		pathPub.publish(pathMsg);
		ROS_INFO("Path found and published!");
	}

int main(int argc, char **argv) {
	ros::init(argc, argv, "astar_planner");

	AStar* planner = new AStar();
	return 0;
}


			// if (cur_x - 1 >= 0){
			// 	if (cur_y - 1 >= 0){
			// 		AStarNode node1(cur_x - 1, cur_y - 1, 0);
			// 		AStarNode node2(cur_x - 1, cur_y, 0);
			// 		AStarNode node3(cur_x, cur_y - 1, 0);

			// 		neighbors.push_back(node1);
			// 		neighbors.push_back(node2);
			// 		neighbors.push_back(node3);
			// 	} 
			// 	if (cur_y + 1 < mapGridHeight){
			// 		AStarNode node1(cur_x - 1, cur_y + 1, 0);
			// 		AStarNode node2(cur_x - 1, cur_y, 0);
			// 		AStarNode node3(cur_x, cur_y + 1, 0);

			// 		neighbors.push_back(node1);
			// 		neighbors.push_back(node2);
			// 		neighbors.push_back(node3);
			// 	}
			// } 
			
			// if (cur_x + 1 < mapGridWidth){
			// 	if (cur_y - 1 >= 0){
			// 		AStarNode node1(cur_x + 1, cur_y - 1, 0);
			// 		AStarNode node2(cur_x + 1, cur_y, 0);
			// 		AStarNode node3(cur_x, cur_y - 1, 0);

			// 		neighbors.push_back(node1);
			// 		neighbors.push_back(node2);
			// 		neighbors.push_back(node3);
			// 	} 
			// 	if (cur_y + 1 < mapGridHeight){
			// 		AStarNode node1(cur_x + 1, cur_y + 1, 0);
			// 		AStarNode node2(cur_x + 1, cur_y, 0);
			// 		AStarNode node3(cur_x, cur_y + 1, 0);

			// 		neighbors.push_back(node1);
			// 		neighbors.push_back(node2);
			// 		neighbors.push_back(node3);
			// 	}
			// }
