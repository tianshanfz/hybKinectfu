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

__global__ void integrateKernel(const DepthfMap2D input_depthmap,
								const Vector4fMap2D input_normalmap,
								const Color3uMap2D input_colormap,
								tsdfvolume volume,
								const IntegrateParams integrate_params,
								bool has_color,
								bool color_angled,
								const Mat44 transform_inv,
								const CameraParams depth_camera_params,
								const CameraParams rgb_camera_params,
								unsigned *updated_voxel_count
								)
{
	//hashMap.integrate(blockIdx);
	const unsigned  x = threadIdx.x + blockIdx.x * blockDim.x;
	const unsigned  y = threadIdx.y + blockIdx.y * blockDim.y;
	if (x >= volume.resolution().x || y >= volume.resolution().y) return;
	atomicAdd(updated_voxel_count, 1); // Atomic addition
	for (unsigned z = 0; z < volume.resolution().z; z++)
	{
		int3 pi = { (int)x, (int)y, (int)z };
		float3 pf_world = volume.voxelPosToWorld(pi);
		float4 pf4 = transform_inv*make_float4(pf_world.x,pf_world.y,pf_world.z, 1.0);
		float3 pf=make_float3(pf4.x,pf4.y,pf4.z);

		if (pf.z <= 0) continue;

		int2 screen_pos_depth = DepthCamera::projectSkeletonToScreen(pf,depth_camera_params);
		if (screen_pos_depth.x >= depth_camera_params.cols-1 || screen_pos_depth.y >= depth_camera_params.rows-1 || screen_pos_depth.x < 1 || screen_pos_depth.y < 1)
		{

			continue;
		}

		float depth = input_depthmap.at(screen_pos_depth.x,screen_pos_depth.y);
		if (depth == 0) continue;
		float normalz=	input_normalmap.at(screen_pos_depth.x, screen_pos_depth.y).z;
		int2 screen_pos_color;
		uchar3 color;
		if(has_color)
		{
			screen_pos_color.x = int(pf.x *525 / pf.z + 320);
			screen_pos_color.y = int( pf.y * 525/ pf.z + 240);
			color= input_colormap.at(screen_pos_color.x, screen_pos_color.y);
			if (screen_pos_color.x >= rgb_camera_params.cols-1 || screen_pos_color.y >= rgb_camera_params.rows-1 || screen_pos_color.x < 1 || screen_pos_color.y < 1)
			{
				continue;
			}
		}
		if (depth < integrate_params.fMaxIntegrateDist)
		{
			float sdf = depth - pf.z;
			if (sdf > -integrate_params.fSdfTruncation)
			{
		//		atomicAdd(updated_count, 1); // Atomic addition
				float tsdf= fminf(1.0, sdf / integrate_params.fSdfTruncation);
				float wrk = 1;
				float wrkColor = (color_angled?fminf(1.0,abs(normalz)/0.75):1.0)*2.0;
				volume.updateVoxel(x, y, z, tsdf, wrk, color, wrkColor);
			}
		}
	}
}
void cudaIntegrateVolume(bool has_color,bool use_angle_weight_color,const Mat44& transform,const IntegrateParams& integrate_params,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params)
{
	DepthfMap2D input_depth=CudaDeviceDataMan::instance()->trunced_depth;
	Vector4fMap2D input_normal=CudaDeviceDataMan::instance()->new_normals_pyramid[0];
	Color3uMap2D input_color=CudaDeviceDataMan::instance()->raw_rgb;
	tsdfvolume volume=CudaDeviceDataMan::instance()->volume;
	Mat44 transform_inv=transform.getInverse();
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(volume.resolution().x, BLOCK_SIZE_2D_X), divUp(volume.resolution().y, BLOCK_SIZE_2D_Y));
	unsigned *updated_voxel_count;
	cudaMalloc(&updated_voxel_count,sizeof(unsigned));
	cudaMemset((void*)updated_voxel_count,0,sizeof(unsigned));
	integrateKernel << <gridSize, blockSize >> >(input_depth,input_normal,input_color,volume,integrate_params,has_color,use_angle_weight_color,transform_inv,depth_camera_params,rgb_camera_params,updated_voxel_count);
	cudaDeviceSynchronize();
	int updated_voxel_count_cpu;
	cudaMemcpy(&updated_voxel_count_cpu,updated_voxel_count,sizeof(unsigned),cudaMemcpyDeviceToHost);
	cout<<"integrate update voxels count: "<<updated_voxel_count_cpu<<endl;
	cudaFree(updated_voxel_count);

}
