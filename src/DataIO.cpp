/*
 * DataIO.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: hanyinbo
 */

#include "DataIO.h"
#include"cuda/cuda_declar.h"
void DataIO::saveVector4fMap2D(const Vector4fMap2D& vector4_map,const char* filename)
{
	Vector4fMap2D data_cpu=vector4_map.clone(CPU);
	int cols = data_cpu.cols();
	int rows = data_cpu.rows();
	ofstream file;
	file.open(filename);
	file<<cols<<" "<<rows<<endl;
	for (int row = 0; row < rows; row++)
	for (int col = 0; col < cols; col++)
	{
		file<<data_cpu.at(col,row).x<<" "
			<<data_cpu.at(col,row).y<<" "
			<<data_cpu.at(col,row).z<<" "
			<<data_cpu.at(col,row).w<<endl;
	}
	file.close();

}

//static void loadVector4fMap2D(const Vector4fMap2D& normals_map,const char* filename);
void DataIO::saveColor3uMap2D(const Color3uMap2D& colors_map,const char* filename)
{

}
