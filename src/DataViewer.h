/*
 * DataViewer.h
 *
 *  Created on: May 29, 2016
 *      Author: hanyinbo
 */

#ifndef DATAVIEWER_H_
#define DATAVIEWER_H_

#include"utils/cpu_include.h"
#include"cuda/DataMap.h"
class DataViewer {
public:
	static void viewNormal(const Vector4fMap2D& normals_map,const char* window_name);
	//static void viewVertices(const Point4fMap2D& vertices_map,const char* window_name);
	static void viewColors(const Color3uMap2D& colors_map,const char* window_name);
	static void viewDepths(const DepthfMap2D& depths_map,const char* window_name);

};

#endif /* DATAVIEWER_H_ */
