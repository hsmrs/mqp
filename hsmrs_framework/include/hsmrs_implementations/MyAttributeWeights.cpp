#include <hsmrs_framework/AttributeWeights.h>

class MyAttributeWeights: public AttributeWeights
{
public:
	MyAttributeWeights()
	{
		attrMap = new std::map<std::string, double>();
		(*attrMap)["attr1"] = 1.0;
		(*attrMap)["attr2"] = 0.5;
	}

	double getWeight(std::string name)
	{
		return (*attrMap)[name];
	}

	std::map<std::string, double> getWeights()
	{
		return std::map<std::string, double>(*attrMap);
	}

	~MyAttributeWeights()
	{
		delete attrMap;
	}
private:
	std::map<std::string, double>* attrMap;
};
