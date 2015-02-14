#ifndef _ASTAR_NODE_H_
#define _ASTAR_NODE_H_

class AStarNode {
private:
	int x;
	int y;
	int cost;
	AStarNode *parent;

public:
	bool hasParent;

	AStarNode() {
		this->x = 0;
		this->y = 0;
		this->cost = 0;
		hasParent = false;
	}

	AStarNode(int x, int y, int cost) {
		this->x = x; 
		this->y = y; 
		this->cost = cost;
		hasParent = false;
	}

	void setX(int x) {this->x = x;}

	void setY(int y) {this->y = y;}

	void setCost(int cost) {this->cost = cost;}

	void setParent(AStarNode param_parent){
		hasParent = true;
		parent = new AStarNode;
		*parent = param_parent;
	}

	int getX() const {return x;}

	int getY() const {return y;}

	int getCost() const {return cost;}

	AStarNode* getParent(){return parent;}


	void incrementCost(int amount) {cost += amount;}

	bool operator< (const AStarNode &node) const {return cost < node.getCost();}

	bool operator> (const AStarNode &node) const {return cost > node.getCost();}

	bool operator== (const AStarNode &node) const {return x == node.getX() && y == node.getY() && cost == node.getCost();}

};

#endif