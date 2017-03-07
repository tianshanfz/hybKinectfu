/* *
 * Copyright 1993-2012 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 */
#include <stdio.h>
#include <stdlib.h>
#include"CudaWrappers.h"
#include "CudaDeviceDataMan.h"


__global__ void pryDownNormalsKernel(const Vector4fMap2D input,Vector4fMap2D output)
{
	const unsigned  x = threadIdx.x + blockIdx.x * blockDim.x;
	const unsigned  y = threadIdx.y + blockIdx.y * blockDim.y;
	const unsigned  cols = input.cols();
	const unsigned  rows = input.rows();
	if (x >=cols || y >=rows)return;
	output.at(x, y)= { 0, 0, 0, 0 };
	const unsigned  xs = x * 2;
	const unsigned  ys = y * 2;
	float4 p00 = input.at(xs,ys);
	float4 p01 = input.at(xs+1, ys);
	float4 p10 = input.at(xs, ys+1);
	float4 p11 = input.at(xs+1, ys+1);

	if (isZero(p01) || isZero(p10) || isZero(p00) || isZero(p11)) return;
	float4 n=(p00 +p01 + p10 +p11)*0.25;
	float3 n_normed=normalize(make_float3(n.x,n.y,n.z));
	output.at(x, y)= make_float4(n_normed.x,n_normed.y,n_normed.z,0);
}

__global__ void pryDownVerticesKernel(const Point4fMap2D input,Point4fMap2D output)
{
	const unsigned  x = threadIdx.x + blockIdx.x * blockDim.x;
	const unsigned  y = threadIdx.y + blockIdx.y * blockDim.y;
	const unsigned  cols = input.cols();
	const unsigned  rows = input.rows();
	if (x >=cols || y >=rows)return;
	const unsigned  xs = x * 2;
	const unsigned  ys = y * 2;

	float4 p00 = input.at(xs,ys);
	float4 p01 = input.at(xs+1, ys);
	float4 p10 = input.at(xs, ys+1);
	float4 p11 = input.at(xs+1, ys+1);

	if (p00.z == 0 || p01.z == 0 || p10.z == 0 || p11.z == 0)
	{
		output.at(x, y)= { 0, 0, 0, 0 };
	}
	else
	{
		output.at(x, y)= (p00 +p01 + p10 +p11)*0.25;
	}
}


void cudaDownSampleNewVertices()
{
	int levels=CudaDeviceDataMan::instance()->new_vertices_pyramid.size();
	for(int i=0;i<levels-1;i++)
	{
		Point4fMap2D input=CudaDeviceDataMan::instance()->new_vertices_pyramid[i];
		Point4fMap2D output=CudaDeviceDataMan::instance()->new_vertices_pyramid[i+1];
		const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
		const dim3 gridSize(divUp(output.cols(), BLOCK_SIZE_2D_X), divUp(output.rows(), BLOCK_SIZE_2D_Y));
		pryDownVerticesKernel<<<blockSize,gridSize>>>(input,output);
	}
}
void cudaDownSampleNewNormals()
{
	int levels=CudaDeviceDataMan::instance()->new_normals_pyramid.size();
	for(int i=0;i<levels-1;i++)
	{
		Point4fMap2D input=CudaDeviceDataMan::instance()->new_normals_pyramid[i];
		Point4fMap2D output=CudaDeviceDataMan::instance()->new_normals_pyramid[i+1];
		const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
		const dim3 gridSize(divUp(output.cols(), BLOCK_SIZE_2D_X), divUp(output.rows(), BLOCK_SIZE_2D_Y));
		pryDownNormalsKernel<<<blockSize,gridSize>>>(input,output);
	}
}
void cudaDownSampleModelVertices()
{
	int levels=CudaDeviceDataMan::instance()->new_vertices_pyramid.size();
	for(int i=0;i<levels-1;i++)
	{
		Point4fMap2D input=CudaDeviceDataMan::instance()->model_vertices_pyramid[i];
		Point4fMap2D output=CudaDeviceDataMan::instance()->model_vertices_pyramid[i+1];
		const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
		const dim3 gridSize(divUp(output.cols(), BLOCK_SIZE_2D_X), divUp(output.rows(), BLOCK_SIZE_2D_Y));
		pryDownVerticesKernel<<<blockSize,gridSize>>>(input,output);
		cudaDeviceSynchronize();
	}
}
 void cudaDownSampleModelNormals()
 {
	 int levels=CudaDeviceDataMan::instance()->new_normals_pyramid.size();
	for(int i=0;i<levels-1;i++)
	{
		Point4fMap2D input=CudaDeviceDataMan::instance()->model_normals_pyramid[i];
		Point4fMap2D output=CudaDeviceDataMan::instance()->model_normals_pyramid[i+1];
		const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
		const dim3 gridSize(divUp(output.cols(), BLOCK_SIZE_2D_X), divUp(output.rows(), BLOCK_SIZE_2D_Y));
		pryDownNormalsKernel<<<blockSize,gridSize>>>(input,output);
		cudaDeviceSynchronize();
	}
 }
