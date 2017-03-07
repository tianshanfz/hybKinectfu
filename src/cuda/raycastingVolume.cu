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

__device__
bool gradientForPoint(const tsdfvolume& volume, const float3& samplepos, const float3& vertex_found, float3& grad)
{
	int3  g = volume.worldPosToVoxel(samplepos);
	if (g.x <= 1 || g.x >= volume.resolution().x - 2)return false;
	if (g.y <= 1 || g.y >= volume.resolution().y - 2)return false;
	if (g.z <= 1 || g.z >= volume.resolution().z - 2)return false;
	float3 cell_size;
	cell_size.x = volume.size().x / volume.resolution().x;
	cell_size.y = volume.size().y / volume.resolution().y;
	cell_size.z = volume.size().z / volume.resolution().z;
	float3 n;
	float fx1,fx2;
	if (false == volume.interpolateSDF(make_float3(vertex_found.x+cell_size.x,vertex_found.y,vertex_found.z), fx1)) return false;
	if (false == volume.interpolateSDF(make_float3(vertex_found.x-cell_size.x,vertex_found.y,vertex_found.z), fx2)) return false;
	n.x = (fx1 - fx2);
	float fy1,fy2;
	if (false == volume.interpolateSDF(make_float3(vertex_found.x,vertex_found.y+cell_size.y,vertex_found.z), fy1)) return false;
	if (false == volume.interpolateSDF(make_float3(vertex_found.x,vertex_found.y-cell_size.y,vertex_found.z), fy2)) return false;
	n.y = (fy1-fy2);
	float fz1,fz2;
	if (false == volume.interpolateSDF(make_float3(vertex_found.x,vertex_found.y,vertex_found.z+cell_size.z), fz1)) return false;
	if (false == volume.interpolateSDF(make_float3(vertex_found.x,vertex_found.y,vertex_found.z-cell_size.z), fz2)) return false;
	n.z = (fz1-fz2);
	float len=norm(n);
	if (len <1e-8) return false;
	grad = n*(1/len);
	return true;
}

__device__ __forceinline__ float
getMinTime(const float3& volume_max, const float3& origin, const float3& dir)
{
	float txmin = ((dir.x > 0 ? 0.f : volume_max.x) - origin.x) / dir.x;
	float tymin = ((dir.y > 0 ? 0.f : volume_max.y) - origin.y) / dir.y;
	float tzmin = ((dir.z > 0 ? 0.f : volume_max.z) - origin.z) / dir.z;

	return fmaxf(fmaxf(txmin, tymin), tzmin);
}

__device__ __forceinline__ float
getMaxTime(const float3& volume_max, const float3& origin, const float3& dir)
{
	float txmax = ((dir.x > 0 ? volume_max.x : 0.f) - origin.x) / dir.x;
	float tymax = ((dir.y > 0 ? volume_max.y : 0.f) - origin.y) / dir.y;
	float tzmax = ((dir.z > 0 ? volume_max.z : 0.f) - origin.z) / dir.z;

	return fminf(fminf(txmax, tymax), tzmax);
}

__device__ void raySample(unsigned  x, unsigned  y,bool has_color,
												const RayCasterParams& raycast_params,
												const CameraParams& depth_camera_params,
												const tsdfvolume& volume,
												const float3& world_camera_pos,
												const float3& world_dir,
												const float3& cam_dir,
												Point4fMap2D& vertices_output,
												Vector4fMap2D& normals_output,
												Color3uMap2D& color_output,
												float min_interval,
												float max_interval
												)
{//must be 0<=x<cols and 0<=y<rows
	float ray_current = min_interval;
	float ray_end = max_interval;
	float last_sdf = 0;
	float3 last_world_pos = { 0, 0, 0 };
	while (ray_current < ray_end)
	{
		float3 cur_world_pos = world_camera_pos + world_dir*ray_current; // sample along ray direction
		Voxel v;

		volume.getVoxel(cur_world_pos, v);

		float sdf = v.tsdf;
	//	if(last_sdf<0&&sdf>0)break;
		if (last_sdf > 0.0f&&sdf < 0.0f) // current sample is always valid here zero crossing
		{

			float ftdt, ft;
			if (false == volume.interpolateSDF(cur_world_pos, ftdt)) break;
			if (false == volume.interpolateSDF(last_world_pos, ft)) break;
			float alpha = ray_current - raycast_params.fRayIncrement*ftdt / (ftdt - ft);
			float3 vertex = world_camera_pos + world_dir*alpha;
			float3 grad;
			if(has_color)
			{
				uchar3 color;
				volume.interpolateColor(vertex, color);
				color_output.at(x, y)= color;
			}
			//float depth = alpha *cam_dir.z;//from ray dir to z dir
			if (false == gradientForPoint(volume, last_world_pos, vertex, grad)) break;
			//atomicAdd(traverse_ray_count, 1); //for debugging
			vertices_output.at(x, y)= make_float4(vertex.x,vertex.y,vertex.z, 1.0);
			normals_output.at(x, y)=make_float4(grad.x,grad.y,grad.z, 0);
			break;
		}
		last_sdf  = sdf;
		last_world_pos = cur_world_pos;
		ray_current += raycast_params.fRayIncrement;
	}
	return;
}

__global__ void raycastKernel(bool has_color,
		const RayCasterParams raycast_params,
		const CameraParams depth_camera_params,
		const Mat44 transform,
		const tsdfvolume volume,
		Point4fMap2D vertices_output,
		Vector4fMap2D normals_output,
		Color3uMap2D color_output,
		float near_plane,
		float far_plane
		)
{
	const unsigned  x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned  y = blockIdx.y*blockDim.y + threadIdx.y;
	const unsigned  cols = depth_camera_params.cols;
	const unsigned  rows = depth_camera_params.rows;
	if(x>=cols||y>=rows) return;
	vertices_output.at(x,y)={ 0, 0, 0,0 };
	normals_output.at(x, y)={ 0, 0, 0, 0 };
	if(has_color)color_output.at(x, y)= { 0, 0, 0 };
	float3 cam_dir = normalize(DepthCamera::depthToSkeleton(x, y, 1.0,depth_camera_params));
	float3 world_camera_pos = transform.getTranslation();
	float4 world_dir4=transform*make_float4(cam_dir.x,cam_dir.y,cam_dir.z, 0.0);
	float3 world_dir = make_float3(world_dir4.x,world_dir4.y,world_dir4.z);
	world_dir.x = (world_dir.x == 0.f) ? 1e-15 : world_dir.x;
	world_dir.y = (world_dir.y == 0.f) ? 1e-15 : world_dir.y;
	world_dir.z = (world_dir.z == 0.f) ? 1e-15 : world_dir.z;
	float min_interval = getMinTime(volume.size(), world_camera_pos, world_dir);
	float max_interval = getMaxTime(volume.size(), world_camera_pos, world_dir);

	min_interval=fmaxf(min_interval,near_plane/cam_dir.z);
	max_interval=fminf(max_interval,far_plane/cam_dir.z);
	if (min_interval >=max_interval) return;

	raySample(x, y,has_color, raycast_params, depth_camera_params, volume, world_camera_pos, world_dir, cam_dir, vertices_output, normals_output, color_output, min_interval, max_interval);
}

void cudaRaycastingVolume(bool has_color,const Mat44& transform,const RayCasterParams& raycast_params,const CameraParams& depth_camera_params,float near_plane,float far_plane)
{
	Point4fMap2D output_vertices=CudaDeviceDataMan::instance()->model_vertices_pyramid[0];
	Vector4fMap2D output_normal=CudaDeviceDataMan::instance()->model_normals_pyramid[0];
	Color3uMap2D output_color=CudaDeviceDataMan::instance()->raycast_rgb;
	tsdfvolume volume=CudaDeviceDataMan::instance()->volume;

	//unsigned *traverse_ray_count=CudaDeviceDataMan::instance()->_debug_count;
	//cudaMemset((void*)traverse_ray_count,0,sizeof(unsigned));
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(output_vertices.cols(), BLOCK_SIZE_2D_X), divUp(output_vertices.rows(), BLOCK_SIZE_2D_Y));

	raycastKernel<<<gridSize,blockSize>>>( has_color,raycast_params,depth_camera_params,transform, volume,output_vertices,output_normal,output_color,near_plane,far_plane);
	cudaDeviceSynchronize();
	//int traverse_ray_count_cpu;
	//cudaMemcpy(&traverse_ray_count_cpu,traverse_ray_count,sizeof(int),cudaMemcpyDeviceToHost);
	//cout<<"traverse ray count: "<<traverse_ray_count_cpu<<endl;

}
