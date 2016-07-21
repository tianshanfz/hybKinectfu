/*
 * MeshGenerator.h
 *
 *  Created on: Jun 13, 2016
 *      Author: hanyinbo
 */

#ifndef MESHGENERATOR_H_
#define MESHGENERATOR_H_
#include"utils/cpu_include.h"
class MeshGenerator {
public:
//	MeshGenerator(){}
//	virtual ~MeshGenerator(){}
	virtual void generateMesh()=0;
	virtual bool saveMesh(const string& filename)=0;
};

#endif /* MESHGENERATOR_H_ */
