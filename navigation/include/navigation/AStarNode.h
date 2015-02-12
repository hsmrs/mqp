#ifndef _ASTAR_NODE_H_
#define _ASTAR_NODE_H_

class AStarNode {
private:
	int x;
	int y;
	int cost;

public:

	AStarNode() {}

	AStarNode(int x, int y, int cost) {this->x = x; this->y = y; this->cost = cost;}

	void setX(int x) {this->x = x;}

	void setY(int y) {this->y = y;}

	void setCost(int cost) {this->cost = cost;}

	int getX() const {return x;}

	int getY() const {return y;}

	int getCost() const {return cost;}

	void incrementCost(int amount) {cost += amount;}

	bool operator< (const AStarNode &node) const {return cost < node.getCost();}

	bool operator> (const AStarNode &node) const {return cost > node.getCost();}

	bool operator== (const AStarNode &node) const {return x == node.getX() && y == node.getY() && cost == node.getCost();}

};

#endif