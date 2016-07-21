#include "CudaWrappers.h"
#include "CudaDeviceDataMan.h"
#include "DepthCamera.h"
#include "device_functions.h"


__host__ __device__ void buildPointToPlaneSolverRows(const float3& p,const float3& q, const float3& n, float output[7])
{
	output[0] = q.y*n.z - q.z*n.y;
	output[1] = q.z*n.x - q.x*n.z;
	output[2] = q.x*n.y - q.y*n.x;
	output[3] = n.x;
	output[4] = n.y;
	output[5] = n.z;
	output[6] = dot(n, p - q);
}
__global__ void computeGbufKernel(const Point4fMap2D input_v,
									const Vector4fMap2D input_n,
									const Point4fMap2D target_v,
									DataMap2D<float> gbuf)
{//precondition :  input_v,target_v are on the same view
	const unsigned  x = blockDim.x*blockIdx.x + threadIdx.x;
	const unsigned  y = blockDim.y*blockIdx.y + threadIdx.y;
	__shared__ float smem[BLOCK_SIZE_2D_X*BLOCK_SIZE_2D_Y];
	const unsigned  cols=input_v.cols();
	const unsigned  rows=input_v.rows();
	//note : don't return here when x>=cols or y>=rows for reduce
	unsigned tid = flattenedThreadId();
	smem[tid] = 0;
	float3 q = make_float3(0,0,0);
	float3 p = make_float3(0, 0, 0);
	float3 n = make_float3(0, 0, 0);

	if (x< cols&& y <rows)
	{
		float4 q4=target_v.get_data(x, y);
		q = make_float3(q4.x,q4.y,q4.z);
		float4 p4=input_v.get_data(x, y);
		p = make_float3(p4.x,p4.y,p4.z);
		float4 n4=input_n.get_data(x, y);
		n = make_float3(n4.x,n4.y,n4.z);
	}
	float row[7] = { 0, 0, 0, 0, 0, 0 ,0};
	if (q.z != 0 && isZero(n)==false)
	{
		buildPointToPlaneSolverRows(p,q, n, row);
	}
	unsigned blockId = blockIdx.x + gridDim.x * blockIdx.y;
	unsigned shift = 0;
	for (int i = 0; i < 6; ++i)        //rows
	{
#pragma unroll
		for (int j = i; j < 7; ++j)          // cols , j==6 indicates b
		{
			__syncthreads();
			smem[tid] = row[i] * row[j];
			__syncthreads();
			reduce(smem, BLOCK_SIZE_2D_X*BLOCK_SIZE_2D_Y);
			if (tid == 0)
			{
				gbuf.set_data(blockId, shift++,smem[0]);
			}
		}
	}

}

void cudaCalPointToPlaneErrSolverParams( unsigned pyramid_level)
{
	Point4fMap2D input_v=CudaDeviceDataMan::instance()->_model_vertices_pyramid[pyramid_level];
	Vector4fMap2D input_n=CudaDeviceDataMan::instance()->_model_normals_pyramid[pyramid_level];
	Point4fMap2D target_v=CudaDeviceDataMan::instance()->_corrs_vertices_pyramid[pyramid_level];
	DataMap2D<float> icp_buf_temp=CudaDeviceDataMan::instance()->_rigid_align_buf_temp_pyramid[pyramid_level];
	DataMap1D<float> icp_buf_reduced=CudaDeviceDataMan::instance()->_rigid_align_buf_reduced;
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input_v.cols(), BLOCK_SIZE_2D_X), divUp(input_v.rows(), BLOCK_SIZE_2D_Y));
	computeGbufKernel << <gridSize, blockSize >> >(input_v, input_n, target_v, icp_buf_temp);
	cudaDeviceSynchronize();
	reduceGbufKernel << <icp_buf_reduced.cols(), 512, 512 * sizeof(float) >> >(icp_buf_temp.data(), icp_buf_temp.cols(),icp_buf_temp.rows(),icp_buf_reduced.data(), 512);

}
