/*
 * CudaWrappers.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef CUDAWRAPPERS_H_
#define CUDAWRAPPERS_H_


#include "DataMap.h"
#include"../AppParams.h"
#include"Mat.h"
/*typedef struct float_vector2
{
	float u;
	float v;
}float_vector2;*/
//extern "C" void cudaBilinearFilterDepth(const DepthfMap<float>& input, DepthfMap<float>& output);
//extern "C" void DownSampleScalar(const float* input, float* output, int coarseSize,int fineSize);
extern "C" void cudaDownSampleNewVertices();
extern "C" void cudaDownSampleNewNormals();
extern "C" void cudaDownSampleModelVertices();
extern "C" void cudaDownSampleModelNormals();

extern "C" void cudaCalculateNewVertices(const CameraParams& depth_camera_params);
extern "C" void cudaCalculateNewNormals();
extern "C" void cudaTruncDepth(float trunc_min,float trunc_max);
extern "C" void cudaBiliearFilterDepth(float sigma_pixel,float sigma_depth);
//extern "C" void cudaProjectionMapFindCorrs(unsigned pyramid_level,const Mat44& cur_transform,const Mat44& last_transform_inv,const CameraParams& depth_camera_params,float dist_thres,float norm_sin_thres);
extern "C" void cudaCalPointToPlaneErrSolverParams( unsigned pyramid_level,const Mat44& cur_transform,const Mat44& last_transform_inv,const CameraParams& depth_camera_params,float dist_thres,float norm_sin_thres);
extern "C" void cudaCalSDFSolverParams(const CameraParams& depth_camera_params,const Mat44& cur_transform);
extern "C" void cudaIntegrateVolume(bool has_color,bool use_angle_weight_color,const Mat44& transform,const IntegrateParams& integrate_params,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params);
extern "C" void cudaRaycastingVolume(bool has_color,const Mat44& transform,const RayCasterParams& raycast_params,const CameraParams& depth_camera_params,float near_plane,float far_plane);
extern "C" void cudaMarchingcube(bool has_color,float threshold_marchingcube);
#endif /* CUDAWRAPPERS_H_ */
