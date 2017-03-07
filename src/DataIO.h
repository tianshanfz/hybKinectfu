/*
 * DataIO.h
 *
 *  Created on: Jun 27, 2016
 *      Author: hanyinbo
 */

#ifndef DATAIO_H_
#define DATAIO_H_

#include"utils/cpu_include.h"
#include"cuda/DataMap.h"
typedef float4 Vector4f;
typedef uchar3 Color3u;
typedef CudaMap2D<Color3u> Color3uMap2D;
typedef CudaMap2D<Vector4f> Vector4fMap2D;
class DataIO {
public:
	static void saveVector4fMap2D(const Vector4fMap2D& vector4_map,const char* filename);
	//static void loadVector4fMap2D(const Vector4fMap2D& normals_map,const char* filename);
	static void saveColor3uMap2D(const Color3uMap2D& colors_map,const char* filename);
	//static void loadVector4fMap2D(const Color3uMap2D& colors_map,const char* filename);

};

#endif /* DATAIO_H_ */
