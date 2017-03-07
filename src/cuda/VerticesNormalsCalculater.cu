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

__global__ void depthToVerticesKernel(const DepthfMap2D input,Point4fMap2D output,const CameraParams depth_camera_params)
{//calculate verticesmap from depthsmap return (0,0,0,0) if invalid
	const unsigned  x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned  y = blockIdx.y*blockDim.y + threadIdx.y;
	const unsigned cols = input.cols();
	const unsigned rows = input.rows();
	if (x >=cols || y >=rows)return;
	float depth=input.at(x,y);
	if (depth == 0)
	{
		output.at(x,y)= make_float4(0, 0, 0, 0);
	}
	else
	{
		float3 v=DepthCamera::depthToSkeleton(x, y, depth,depth_camera_params);
		output.at(x,y)=make_float4(v.x,v.y,v.z,1.0);
	}
}


__global__ void verticesToNormalsKernel(const Point4fMap2D input,Vector4fMap2D output)
{//calculate normalmap from verticesmap return (0,0,0,0) if invalid
	const int  x = threadIdx.x + blockIdx.x * blockDim.x;
	const int  y = threadIdx.y + blockIdx.y * blockDim.y;
	const int  cols = input.cols();
	const int  rows = input.rows();
	if (x >= cols || y >= rows)return;
	output.at(x,y)=make_float4(0,0,0,0);
	if (x == cols - 1 || y == rows - 1||x == 0 || y == 0)return;
	float4 v0, v_up4, v_down4, v_left4, v_right4;
	v0 = input.at(x, y);
	if(v0.z==0)return ;

	v_right4 = input.at(x + 1, y);
	if(v_right4.z==0)return ;
	float3 v_right=make_float3(v_right4.x,v_right4.y,v_right4.z);

	v_up4 = input.at(x, y + 1);
	if(v_up4.z==0)return ;
	float3 v_up=make_float3(v_up4.x,v_up4.y,v_up4.z);

	v_left4 = input.at(x-1, y);
	if(v_left4.z==0)return ;
	float3 v_left=make_float3(v_left4.x,v_left4.y,v_left4.z);

	v_down4 = input.at(x, y-1 );
	if(v_down4.z==0)return ;
	float3 v_down=make_float3(v_down4.x,v_down4.y,v_down4.z);

	float3 c = normalize(cross(v_up - v_down, v_right - v_left));
	output.at(x,y)=make_float4(c.x,c.y,c.z,0);
}
void cudaCalculateNewVertices(const CameraParams& depth_camera_params)
{
	DepthfMap2D input=CudaDeviceDataMan::instance()->filtered_depth;
	Point4fMap2D output=CudaDeviceDataMan::instance()->new_vertices_pyramid[0];
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input.cols(), BLOCK_SIZE_2D_X), divUp(input.rows(), BLOCK_SIZE_2D_Y));
	depthToVerticesKernel<<<gridSize,blockSize>>>(input,output, depth_camera_params);
	cudaDeviceSynchronize();
}

void cudaCalculateNewNormals()
{
	Point4fMap2D input=CudaDeviceDataMan::instance()->new_vertices_pyramid[0];
	Vector4fMap2D output=CudaDeviceDataMan::instance()->new_normals_pyramid[0];
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input.cols(), BLOCK_SIZE_2D_X), divUp(input.rows(), BLOCK_SIZE_2D_Y));
	verticesToNormalsKernel<<<gridSize,blockSize>>>(input,output);
	cudaDeviceSynchronize();
}


