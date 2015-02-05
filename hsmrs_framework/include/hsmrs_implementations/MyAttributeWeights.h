#include "hsmrs_framework/AttributeWeights.h"

class MyAttributeWeights: public AttributeWeights
{
public:
	MyAttributeWeights();

	double getWeight(std::string name);

	//TODO this implementation is memory leaky
	std::map<std::string, double>* getWeights();

	~MyAttributeWeights();
private:
	std::map<std::string, double>* attrMap;
};