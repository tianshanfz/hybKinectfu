/*
 * CudaDeviceDataMan.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef CUDADEVICEDATAMAN_H_
#define CUDADEVICEDATAMAN_H_

#include"tsdfVolume.h"
#include"MarchingcubeData.h"
#include "../AppParams.h"
class CudaDeviceDataMan
{ //store all cuda device datas
public:

	static CudaDeviceDataMan* instance()
	{
		static CudaDeviceDataMan value;
		return &value;
	}

	void init()
	{
		const AppParams* params=AppParams::instance();
		raw_depth.resize(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		trunced_depth.resize(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		filtered_depth.resize(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		raw_rgb.resize(params->_rgb_camera_params.cols,params->_rgb_camera_params.rows);
		raycast_rgb.resize(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		//init pyramids
		unsigned levels=params->_icp_params.nPyramidLevels;
		unsigned cols=params->_depth_camera_params.cols,rows=params->_depth_camera_params.rows;
		const int icp_buf_size=27;//(6*7/2)+6
		for(unsigned level=0;level<levels;level++)
		{
			corrs_vertices_pyramid.push_back({GPU,cols,rows});
			new_vertices_pyramid.push_back({GPU,cols,rows});
			new_normals_pyramid.push_back({GPU,cols,rows});
			model_vertices_pyramid.push_back({GPU,cols,rows});
			model_normals_pyramid.push_back({GPU,cols,rows});
			rigid_align_buf_temp_pyramid.push_back({GPU,divUp(cols,BLOCK_SIZE_2D_X)*divUp(rows,BLOCK_SIZE_2D_Y),icp_buf_size});
			cols>>=1;
			rows>>=1;
		}
		rigid_align_buf_reduced.resize(icp_buf_size);
		volume.init(params->_volume_params);
		marchingcube_data.resize(params->_marchingcube_params.uMaxTriangles);

	}

	//be calful to use the variables below ,they are generally GPU data
	tsdfvolume volume;
	MarchingcubeData marchingcube_data;
	CudaMap2D<float> raw_depth;//meter
	CudaMap2D<float> trunced_depth;
	CudaMap2D<float> filtered_depth;
	CudaMap2D<uchar3> raw_rgb;
	vector<CudaMap2D<float4>> new_vertices_pyramid;
	vector<CudaMap2D<float4>> new_normals_pyramid;
	vector<CudaMap2D<float4>> corrs_vertices_pyramid;
	vector<CudaMap2D<float4>> model_vertices_pyramid;
	vector<CudaMap2D<float4>> model_normals_pyramid;
	vector<CudaMap2D<float>> rigid_align_buf_temp_pyramid;
	CudaMap2D<uchar3> raycast_rgb;
	CudaMap1D<float> rigid_align_buf_reduced;
//	unsigned * _debug_count;

protected:
	CudaDeviceDataMan():volume(GPU),raw_depth(GPU),trunced_depth(GPU),filtered_depth(GPU),raw_rgb(GPU),raycast_rgb(GPU),rigid_align_buf_reduced(GPU),marchingcube_data(GPU)
	{

	}
	//virtual ~CudaDeviceDataMan(){release();};

};

#endif /* CUDADEVICEDATAMAN_H_ */
