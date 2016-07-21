/* *
 * Copyright 1993-2012 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 */

#include"CudaWrappers.h"
#include "CudaDeviceDataMan.h"

/**
 * Host function that prepares data array and passes it to the CUDA kernel.
 */
__global__ void trucDepthKernel(DepthfMap2D input,DepthfMap2D output,float trunc_min, float trunc_max)
{
	const unsigned  x = blockIdx.x*blockDim.x + threadIdx.x;
	const unsigned  y = blockIdx.y*blockDim.y + threadIdx.y;
	const unsigned  cols = input.cols();
	const unsigned  rows = input.rows();
	if (x <cols && y <rows)
	{
		float depth=input.get_data(x,y);
		if(depth<trunc_max&&depth>trunc_min)
		{
			output.set_data(x,y,depth);
		}
		else
		{
			output.set_data(x,y,0);
		}
	}

}
__global__ void biateralFilterKernel(const DepthfMap2D input, DepthfMap2D output, float sigma_space2_inv_half, float sigma_data2_inv_half,float sigma_data,float sigma_pixel)
{
	const int  x = blockIdx.x*blockDim.x + threadIdx.x;
	const int  y = blockIdx.y*blockDim.y + threadIdx.y;
	int cols = input.cols();
	int rows = input.rows();
	if (x >=cols || y >=rows)return;
	output.set_data(x, y, 0);
	const int kernelRadius = (int)ceil(2.0*sigma_pixel);
	float value = input.get_data(x, y);
	if (value == 0)
	{
		return;
	}
	int x_start = max(x - kernelRadius, 0);
	int x_end = min(x +kernelRadius, cols -1);
	int y_start = max(y - kernelRadius, 0);
	int y_end = min(y + kernelRadius, rows - 1);
	float sum1 = 0;
	float sum2 = 0;
	for (int cy = y_start; cy <= y_end; ++cy)
	{
		for (int cx =x_start; cx <=x_end; ++cx)
		{
			float tmp = input.get_data(cx, cy);
			if (tmp == 0)
			{
				continue;
			}
			if(abs(tmp-value)>5*sigma_data)
			{
				return;
			}
			float space2 = (x - cx) * (x - cx) + (y - cy) * (y - cy);
			float data2 = (value - tmp) * (value - tmp);
			float weight = __expf(-(space2 * sigma_space2_inv_half + data2 * sigma_data2_inv_half));
			sum1 += tmp * weight;
			sum2 += weight;
		}
	}
	//must be sum2 >0 unless sigma_pixel<0
	output.set_data(x, y, sum1 / sum2);
}
void cudaTruncDepth(float trunc_min,float trunc_max)
{
	DepthfMap2D input=CudaDeviceDataMan::instance()->_raw_depth;
	DepthfMap2D output=CudaDeviceDataMan::instance()->_trunced_depth;
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input.cols(), BLOCK_SIZE_2D_X), divUp(input.rows(), BLOCK_SIZE_2D_Y));
	trucDepthKernel<<<gridSize,blockSize>>>(input,output, trunc_min,trunc_max);
	cudaDeviceSynchronize();
}
void cudaBiliearFilterDepth(float sigma_space,float sigma_data)
{
	DepthfMap2D input=CudaDeviceDataMan::instance()->_trunced_depth;
	DepthfMap2D output=CudaDeviceDataMan::instance()->_filtered_depth;
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input.cols(), BLOCK_SIZE_2D_X), divUp(input.rows(), BLOCK_SIZE_2D_Y));
	float sigmadata_inv = 0.5 / (sigma_data*sigma_data);
	float sigmaspace_inv = 0.5 / (sigma_space*sigma_space);

	biateralFilterKernel<<<gridSize,blockSize>>>(input,output, sigmaspace_inv,sigmadata_inv,sigma_data,sigma_space);
	cudaDeviceSynchronize();
}
