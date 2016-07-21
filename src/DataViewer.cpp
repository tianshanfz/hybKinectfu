/*
 * DataViewer.cpp
 *
 *  Created on: May 29, 2016
 *      Author: hanyinbo
 */

#include "DataViewer.h"
#include<opencv2/opencv.hpp>



void DataViewer::viewNormal(const Vector4fMap2D& normals_map,const char* window_name)
{
	Vector4fMap2D normals_view_map;
	normals_view_map.create_cpu(normals_map);
	float3 minValue = make_float3(-1, -1, -1);
	float3 maxValue = make_float3(1, 1, 1);
	int cols = normals_view_map.cols();
	int rows = normals_view_map.rows();
	unsigned char *arr_normals = new unsigned char[cols*rows * 3];
	for (int row = 0; row < rows; row++)
	for (int col = 0; col < cols; col++)
	{
		arr_normals[col * 3 + 0 + row*cols * 3] = 255 * (normals_view_map.get_data(col, row).x - minValue.x) / (maxValue.x - minValue.x);
		arr_normals[col * 3 + 1 + row*cols * 3] = 255 * (normals_view_map.get_data(col, row).y - minValue.y) / (maxValue.y - minValue.y);
		arr_normals[col * 3 + 2 + row*cols * 3] = 255 * (normals_view_map.get_data(col, row).z - minValue.z) / (maxValue.z - minValue.z);
	}
	cv::Mat image_show(rows, cols, CV_8UC3, (void*)arr_normals);
	cv::imshow(window_name, image_show);
	cv::waitKey(1);
	SAFE_DELETE_ARR(arr_normals);
	normals_view_map.destroy();
}

void DataViewer::viewColors(const Color3uMap2D& colors_map,const char* window_name)
{
	Color3uMap2D colors_view_map;
	colors_view_map.create_cpu(colors_map);
	int cols = colors_view_map.cols();
	int rows = colors_view_map.rows();
	cv::Mat image_show = cv::Mat(rows, cols, CV_8UC3, (void*)colors_view_map.data());
	//cv::Mat mImageBGR;
	//cv::cvtColor(mImageShow, mImageBGR, CV_RGB2BGR);
	cv::imshow(window_name, image_show);
	cv::waitKey(1);
	colors_view_map.destroy();
}
void DataViewer::viewDepths(const DepthfMap2D& depths_map,const char* window_name)
{
	DepthfMap2D depths_view_map;
	depths_view_map.create_cpu(depths_map);
	int cols = depths_view_map.cols();
	int rows = depths_view_map.rows();

	cv::Mat image_show;
	cv::Mat data_mat= cv::Mat(rows, cols, CV_32FC1, (void*)depths_view_map.data());
	double min_val,max_val;
	cv::minMaxIdx(data_mat,&min_val,&max_val);
	data_mat.convertTo( image_show, CV_8U, 255.0 / max_val );
	cv::imshow(window_name, image_show);
	cv::waitKey(1);
	depths_view_map.destroy();
}
