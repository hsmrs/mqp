#include "AttributeWeights.h"

class MyAttributeWeights: public AttributeWeights
{
public:
	MyAttributeWeights::MyAttributeWeights()
	{
		attrMap = new std::map<std::string, double>();
		(*attrMap)["attr1"] = 1.0;
		(*attrMap)["attr2"] = 0.5;
	}

	double AttributeWeights::getWeight(std::string name)
	{
		return (*attrMap)[name];
	}

	//TODO this implementation is memory leaky
	std::map<std::string, double>* AttributeWeights::getWeights()
	{
		return new std::map<std::string, double>(*attrMap);
	}

	MyAttributeWeights::~MyAttributeWeights()
	{
		delete attrMap;
	}
private:
	std::map<std::string, double>* attrMap;
};