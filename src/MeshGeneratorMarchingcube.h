/*
 * MeshGenerator.h
 *
 *  Created on: Jun 6, 2016
 *      Author: hanyinbo
 */

#ifndef MESHGENERATOR_MARCHINGCUBEH_
#define MESHGENERATOR_MARCHINGCUBEH_
#include"MeshGenerator.h"
#include"utils/mesh/meshData.h"
class MeshGeneratorMarchingcube:public MeshGenerator {
public:
	MeshGeneratorMarchingcube();
	virtual ~MeshGeneratorMarchingcube();
	virtual void generateMesh();
	virtual bool saveMesh(const string& filename);

protected:
	bool textureMesh(const string &filename);
	bool copyTrianglesToCPU();
	ml::MeshDataf _meshes;
};

#endif /* MESHGENERATOR_H_ */
