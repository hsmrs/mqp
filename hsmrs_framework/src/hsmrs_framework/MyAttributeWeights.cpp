#include "hsmrs_implementations/MyAttributeWeights.h"

MyAttributeWeights::MyAttributeWeights()
	{
		attrMap = new std::map<std::string, double>();
		(*attrMap)["attr1"] = 1.0;
		(*attrMap)["attr2"] = 0.5;
	}

	double MyAttributeWeights::getWeight(std::string name)
	{
		return (*attrMap)[name];
	}

	//TODO this implementation is memory leaky
	std::map<std::string, double>* MyAttributeWeights::getWeights()
	{
		return new std::map<std::string, double>(*attrMap);
	}

	MyAttributeWeights::~MyAttributeWeights()
	{
		delete attrMap;
	}
