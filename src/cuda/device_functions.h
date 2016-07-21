/*
 * device_functions.h
 *
 *  Created on: May 28, 2016
 *      Author: hanyinbo
 */

#ifndef DEVICE_FUNCTIONS_H_
#define DEVICE_FUNCTIONS_H_

__device__ __forceinline__ unsigned flattenedThreadId()
{
	return threadIdx.z * blockDim.x * blockDim.y + threadIdx.y * blockDim.x + threadIdx.x;
}

template<typename T>
__device__  __forceinline__ void reduce(volatile T* buffer, unsigned n)
{
	unsigned tid = flattenedThreadId();
	T val = buffer[tid];
	if (n >= 1024) { if (tid < 512) buffer[tid] = val = val+ buffer[tid + 512]; __syncthreads(); }
	if (n >= 512) { if (tid < 256) buffer[tid] = val = val+ buffer[tid + 256]; __syncthreads(); }
	if (n >= 256) { if (tid < 128) buffer[tid] = val = val+ buffer[tid + 128]; __syncthreads(); }
	if (n >= 128) { if (tid <  64) buffer[tid] = val = val+ buffer[tid + 64]; __syncthreads(); }
	if (tid < 32)
	{
		if (n >= 64) { buffer[tid] = val = val + buffer[tid + 32]; }
		if (n >= 32) { buffer[tid] = val = val + buffer[tid + 16]; }
		if (n >= 16) { buffer[tid] = val = val + buffer[tid + 8]; }
		if (n >= 8) { buffer[tid] = val = val + buffer[tid + 4]; }
		if (n >= 4) { buffer[tid] = val = val + buffer[tid + 2]; }
		if (n >= 2) { buffer[tid] = val = val + buffer[tid + 1]; }
	}
	//printf("buffer[%d]:%.2f\n", tid, buffer[tid]);
}


template<typename T>
__global__  void reduceGbufKernel(const T* gbuf,unsigned cols,unsigned rows, T* mbuf, int reduce_size)
{
	const T *beg = gbuf+blockIdx.x*cols;
	const T *end = beg + cols;
	int tid = threadIdx.x;

	T sum = 0.f;
	for (const T *t = beg + tid; t < end; t += reduce_size)
		sum += *t;

	extern __shared__ T smem[];
	smem[tid] = sum;
	__syncthreads();
	reduce(smem, reduce_size);
	if (tid == 0)
		mbuf[blockIdx.x]=smem[0];//sum of all row i * row j
}

#endif /* DEVICE_FUNCTIONS_H_ */
