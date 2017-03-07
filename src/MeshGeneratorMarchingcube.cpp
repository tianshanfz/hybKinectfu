/*
 * MeshGeneratorMarchingcube.cpp
 *
 *  Created on: Jun 6, 2016
 *      Author: hanyinbo
 */

#include "MeshGeneratorMarchingcube.h"
#include "utils/mesh/MeshIO.h"
#include "AppParams.h"
#include "cuda/CudaWrappers.h"
#include "cuda/CudaDeviceDataMan.h"
//#include"keyframeMan.h"
MeshGeneratorMarchingcube::MeshGeneratorMarchingcube() {
	// TODO Auto-generated constructor stub

}

MeshGeneratorMarchingcube::~MeshGeneratorMarchingcube() {
	// TODO Auto-generated destructor stub
}

void MeshGeneratorMarchingcube::generateMesh()
{
	cout<<"waiting for marching cube "<<endl;
	cudaMarchingcube(AppParams::instance()->_switch_params.useRGBData ,
			300*AppParams::instance()->_volume_params.fVolumeMeterSize/AppParams::instance()->_volume_params.nResolution);
	cout<<"marching cube done"<<endl;
}
bool MeshGeneratorMarchingcube::copyTrianglesToCPU()
{
	MarchingcubeData marchingcube_data_cpu=CudaDeviceDataMan::instance()->marchingcube_data.clone(CPU);
	int triangle_num=marchingcube_data_cpu.triangleNums();
	cout << "Marching Cubes triangles = " <<triangle_num  << std::endl;
	if (triangle_num == 0)
	{
		return false;
	}
	_meshes.m_Vertices.resize( 3 * triangle_num);
	if (AppParams::instance()->_switch_params.useRGBData == true)
	{
		_meshes.m_Colors.resize(3* triangle_num);
	}

	//float3*  vc = (float3*) triangle;
	for (unsigned int i = 0; i <  triangle_num; i++) {
		const Triangle& triangle=marchingcube_data_cpu.at(i);
		_meshes.m_Vertices[3*i] = triangle.v0.pos;
		_meshes.m_Vertices[3 * i + 1] = triangle.v1.pos;
		_meshes.m_Vertices[3 * i + 2] = triangle.v2.pos;
		if (AppParams::instance()->_switch_params.useRGBData == true)
		{
			_meshes.m_Colors[3 * i] = make_float4(triangle.v0.color.z,triangle.v0.color.y,triangle.v0.color.x, 1.0);
			_meshes.m_Colors[3 * i + 1] = make_float4(triangle.v1.color.z,triangle.v1.color.y,triangle.v1.color.x, 1.0);
			_meshes.m_Colors[3 * i + 2] = make_float4(triangle.v2.color.z,triangle.v2.color.y,triangle.v2.color.x, 1.0);
		}
	}

	return true;
}
bool MeshGeneratorMarchingcube::saveMesh(const string& filename)
{
	if(false==copyTrianglesToCPU())
	{
		cout<<"copy triangles to cpu failed"<<endl;
		return false;

	}
	//create index buffer (required for merging the triangle soup)
	_meshes.m_FaceIndicesVertices.resize(_meshes.m_Vertices.size());
	for (unsigned int i = 0; i < (unsigned int)_meshes.m_Vertices.size() / 3; i++) {
		_meshes.m_FaceIndicesVertices[i][0] = 3 * i + 0;
		_meshes.m_FaceIndicesVertices[i][1] = 3 * i + 1;
		_meshes.m_FaceIndicesVertices[i][2] = 3 * i + 2;
	}
	std::cout << "vertices:\t" << _meshes.m_Vertices.size() << std::endl;
	std::cout << "faces :\t" << _meshes.m_FaceIndicesVertices.size() << std::endl;
	std::cout << "merging close vertices... ";
	_meshes.mergeCloseVertices(0.0001f, true);
	std::cout << "done!" << std::endl;
	std::cout << "vertices e:\t" << _meshes.m_Vertices.size() << std::endl;
	std::cout << "faces :\t" << _meshes.m_FaceIndicesVertices.size() << std::endl;
	std::cout << "removing duplicate faces... ";
	_meshes.removeDuplicateFaces();
	std::cout << "done!" << std::endl;
	_meshes.computeVertexNormals();
	std::cout << "vertices :\t" << _meshes.m_Vertices.size() << std::endl;
	std::cout << "faces :\t" << _meshes.m_FaceIndicesVertices.size() << std::endl;

//	m_meshData.applyTransform(transform);

	std::cout << "saving mesh (" << filename << ") ...";
	ml::MeshIOf::saveToFile(filename, _meshes);
	std::cout << "done!" << std::endl;
	return 0;//textureMesh(filename);

}
