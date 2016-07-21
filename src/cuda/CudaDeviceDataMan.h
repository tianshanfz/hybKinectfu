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
class CudaDeviceDataMan {
public:

	static CudaDeviceDataMan* instance()
	{
		static CudaDeviceDataMan value;
		return &value;
	}
	bool init()
	{
		if(_inited)return false;
		const AppParams* params=AppParams::instance();
		cudaMalloc((void**)&_debug_count,sizeof(unsigned));
		_raw_depth.create_gpu(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		_trunced_depth.create_gpu(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		_filtered_depth.create_gpu(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		_raw_rgb.create_gpu(params->_rgb_camera_params.cols,params->_rgb_camera_params.rows);
		_raycast_rgb.create_gpu(params->_depth_camera_params.cols,params->_depth_camera_params.rows);
		//init pyramids
		unsigned levels=params->_icp_params.nPyramidLevels;
		unsigned cols=params->_depth_camera_params.cols,rows=params->_depth_camera_params.rows;
		const int icp_buf_size=27;//(6*7/2)+6
		_corrs_vertices_pyramid.resize(levels);
		_new_vertices_pyramid.resize(levels);
		_new_normals_pyramid.resize(levels);
		_model_vertices_pyramid.resize(levels);
		_model_normals_pyramid.resize(levels);
		_rigid_align_buf_temp_pyramid.resize(levels);
		for(unsigned level=0;level<levels;level++)
		{
			_corrs_vertices_pyramid[level].create_gpu(cols,rows);
			_new_vertices_pyramid[level].create_gpu(cols,rows);
			_new_normals_pyramid[level].create_gpu(cols,rows);
			_model_vertices_pyramid[level].create_gpu(cols,rows);
			_model_normals_pyramid[level].create_gpu(cols,rows);
			_rigid_align_buf_temp_pyramid[level].create_gpu(divUp(cols,BLOCK_SIZE_2D_X)*divUp(rows,BLOCK_SIZE_2D_Y),icp_buf_size);
			cols>>=1;
			rows>>=1;
		}
		_rigid_align_buf_reduced.create_gpu(icp_buf_size);
		_marchingcube_data.init(params->_marchingcube_params.uMaxTriangles,GPU);
		_inited=_volume.init(params->_volume_params);

		return _inited;
	}

	void release()
	{
		if(!_inited)return;
		cudaFree(_debug_count);
		_marchingcube_data.destroy();
		_raw_depth.destroy();
		_trunced_depth.destroy();
		_filtered_depth.destroy();
		_raw_rgb.destroy();
		_raycast_rgb.destroy();
		//release pyramids
		unsigned levels=_new_vertices_pyramid.size();
		for(unsigned level=0;level<levels;level++)
		{
			_corrs_vertices_pyramid[level].destroy();
			_new_vertices_pyramid[level].destroy();
			_new_normals_pyramid[level].destroy();
			_model_vertices_pyramid[level].destroy();
			_model_normals_pyramid[level].destroy();
			_rigid_align_buf_temp_pyramid[level].destroy();
		}
		_rigid_align_buf_reduced.destroy();
		_volume.release();
		_inited=false;
	}

	//be calful to use the variables below ,they are generally GPU data
	tsdfvolume _volume;
	MarchingcubeData _marchingcube_data;
	DataMap2D<float> _raw_depth;//meter
	DataMap2D<float> _trunced_depth;
	DataMap2D<float> _filtered_depth;
	DataMap2D<uchar3> _raw_rgb;
	vector<DataMap2D<float4>> _new_vertices_pyramid;
	vector<DataMap2D<float4>> _new_normals_pyramid;
	vector<DataMap2D<float4>> _corrs_vertices_pyramid;
	vector<DataMap2D<float4>> _model_vertices_pyramid;
	vector<DataMap2D<float4>> _model_normals_pyramid;
	vector<DataMap2D<float>> _rigid_align_buf_temp_pyramid;
	DataMap2D<uchar3> _raycast_rgb;
	DataMap1D<float> _rigid_align_buf_reduced;
	unsigned * _debug_count;

protected:
	CudaDeviceDataMan():_inited(false){}
	virtual ~CudaDeviceDataMan(){release();};

	bool _inited;
};

#endif /* CUDADEVICEDATAMAN_H_ */
