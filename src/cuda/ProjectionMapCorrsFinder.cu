/* *
 * Copyright 1993-2012 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 */
#include "CudaWrappers.h"
#include "CudaDeviceDataMan.h"
#include "DepthCamera.h"

__global__ void findCorrespondecesKernel(const Point4fMap2D input_vertices,
		const Vector4fMap2D input_normals,
		const Point4fMap2D target_vertices,
		const Vector4fMap2D target_normals,
		const CameraParams depth_camera_params,
		const Mat44 cur_transform,
		const Mat44 last_transform_inv,
		Point4fMap2D corres_vertices,
		float dist_thres,
		float norm_sin_thres
		)
{//precondition:corres_vertices is cleared!!
	//find the corres point of input point on target view
	const unsigned  x = blockDim.x*blockIdx.x + threadIdx.x;
	const unsigned  y = blockDim.y*blockIdx.y + threadIdx.y;
	const unsigned  cols=input_vertices.cols();
	const unsigned  rows=input_vertices.rows();
	if (x >= cols||y>=rows)return;
	float4 input_v_data = input_vertices.at(x, y);
	float4 input_n_data = input_normals.at(x, y);
	if (isZero(input_n_data) ) return;
	float4 v_input_g = cur_transform* input_v_data ;
	float4 n_input_g =cur_transform* input_n_data;
	float4 v_cp = last_transform_inv*v_input_g;

	int2 screen_pos= DepthCamera::projectSkeletonToScreen(make_float3(v_cp.x,v_cp.y,v_cp.z),depth_camera_params);
	if (screen_pos.x < 0 || screen_pos.x >= cols || screen_pos.y < 0 || screen_pos.y >= rows)
	{
		return;
	}

	float4 n_target_g = target_normals.at(screen_pos.x,screen_pos.y);
	if (isZero(n_target_g))return;
	float4 v_target_g = target_vertices.at(screen_pos.x, screen_pos.y);

	float4 delta_g=v_target_g - v_input_g;
	float d = norm(make_float3(delta_g.x,delta_g.y,delta_g.z));
	float d_normal_sin = norm(cross(make_float3(n_target_g.x,n_target_g.y,n_target_g.z), make_float3(n_input_g.x,n_input_g.y,n_input_g.z)));
	if (d <dist_thres&&d_normal_sin<norm_sin_thres)
	{

		float4 pre_corrs_v=corres_vertices.at(screen_pos.x, screen_pos.y);
		if(false==isZero(pre_corrs_v))
		{	//overlap corrs

			float4 delta_pre_g=pre_corrs_v-v_target_g;
			float pre_d=norm(make_float3(delta_g.x,delta_pre_g.y,delta_pre_g.z));
			if(pre_d<d)
			{
	//			atomicAdd(corrs_count, 1);
				return;//use the previous corrs
			}
		}
		corres_vertices.at(screen_pos.x, screen_pos.y)= v_input_g;
	}

}

void cudaProjectionMapFindCorrs(unsigned pyramid_level,
								const Mat44& cur_transform,
								const Mat44& last_transform_inv,
								const CameraParams& depth_camera_params,
								float dist_thres,
								float norm_sin_thres)

{
	Point4fMap2D input_v=CudaDeviceDataMan::instance()->new_vertices_pyramid[pyramid_level];
	Vector4fMap2D input_n=CudaDeviceDataMan::instance()->new_normals_pyramid[pyramid_level];
	Point4fMap2D target_v=CudaDeviceDataMan::instance()->model_vertices_pyramid[pyramid_level];
	Vector4fMap2D target_n=CudaDeviceDataMan::instance()->model_normals_pyramid[pyramid_level];
	Point4fMap2D corrs_v=CudaDeviceDataMan::instance()->corrs_vertices_pyramid[pyramid_level];
	corrs_v.clearData();

//	unsigned * corrs_count=CudaDeviceDataMan::instance()->debug_count;
//	cudaMemset((void*)corrs_count,0,sizeof(unsigned));
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input_v.cols(), BLOCK_SIZE_2D_X), divUp(input_v.rows(), BLOCK_SIZE_2D_Y));
	findCorrespondecesKernel<<<gridSize,blockSize>>>(input_v,
			input_n,
			target_v,
			target_n,
			depth_camera_params,
			cur_transform,
			last_transform_inv,
			corrs_v,
			dist_thres,
			norm_sin_thres
			);
	cudaDeviceSynchronize();
//	int corrs_count_cpu;
//	cudaMemcpy(&corrs_count_cpu,corrs_count,sizeof(unsigned),cudaMemcpyDeviceToHost);
//	cout<<"overlap corrs find count: "<<corrs_count_cpu<<endl;
}
