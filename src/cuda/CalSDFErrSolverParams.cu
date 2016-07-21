#include "CudaWrappers.h"
#include "CudaDeviceDataMan.h"
#include "DepthCamera.h"
#include "device_functions.h"


__device__ bool buildSDFSolverRows(const tsdfvolume& volume,const float3& p, const Mat44& cur_transform, const Mat44& plus_cur_w1, const Mat44&  minus_cur_w1, const Mat44&  plus_cur_w2, const Mat44& minus_cur_w2, const Mat44& plus_cur_w3, const Mat44&  minus_cur_w3,
	float w_h, float v_h, float output[7])
{
	bool ret = true;
	float sdf0;
	float4 pworld0 =cur_transform*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld0.x,pworld0.y,pworld0.z),sdf0))ret=false;
//	else atomicAdd(&delta_n_kesei[6], 1); // Atomic addition
	float4 pworld_r;
	float sdf_w1_plus;  pworld_r = plus_cur_w1*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld_r.x,pworld_r.y,pworld_r.z), sdf_w1_plus))ret = false;
//	else atomicAdd(&delta_n_kesei[0], 1); // Atomic addition
	float sdf_w1_minus;  pworld_r = minus_cur_w1*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld_r.x,pworld_r.y,pworld_r.z), sdf_w1_minus))ret = false;
//	else atomicAdd(&delta_n_kesei[0], -1); // Atomic addition

	float sdf_w2_plus;  pworld_r = plus_cur_w2*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld_r.x,pworld_r.y,pworld_r.z), sdf_w2_plus))ret = false;
//	else atomicAdd(&delta_n_kesei[1], 1); // Atomic addition
	float sdf_w2_minus;  pworld_r = minus_cur_w2*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld_r.x,pworld_r.y,pworld_r.z), sdf_w2_minus))ret = false;
//	else atomicAdd(&delta_n_kesei[1], -1); // Atomic addition

	float sdf_w3_plus;  pworld_r = plus_cur_w3*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld_r.x,pworld_r.y,pworld_r.z), sdf_w3_plus))ret = false;
//	else atomicAdd(&delta_n_kesei[2], 1); // Atomic addition
	float sdf_w3_minus;  pworld_r = minus_cur_w3*make_float4(p.x,p.y,p.z, 1.0);
	if (false == volume.interpolateSDF(make_float3(pworld_r.x,pworld_r.y,pworld_r.z), sdf_w3_minus))ret = false;
//	else atomicAdd(&delta_n_kesei[2], -1); // Atomic addition

	float3 pworld_v;
	float sdf_v1_plus; pworld_v = make_float3(pworld0.x + v_h, pworld0.y, pworld0.z);
	if (false == volume.interpolateSDF(pworld_v, sdf_v1_plus))ret = false;
//	else atomicAdd(&delta_n_kesei[3], 1); // Atomic addition
	float sdf_v1_minus; pworld_v = make_float3(pworld0.x-v_h, pworld0.y, pworld0.z);
	if (false == volume.interpolateSDF(pworld_v, sdf_v1_minus))ret = false;
//	else atomicAdd(&delta_n_kesei[3], -1); // Atomic addition

	float sdf_v2_plus; pworld_v = make_float3(pworld0.x , pworld0.y+v_h, pworld0.z);
	if (false == volume.interpolateSDF(pworld_v, sdf_v2_plus))ret = false;
//	else atomicAdd(&delta_n_kesei[4], 1); // Atomic addition
	float sdf_v2_minus; pworld_v = make_float3(pworld0.x, pworld0.y - v_h, pworld0.z);
	if (false == volume.interpolateSDF(pworld_v, sdf_v2_minus))ret = false;
//	else atomicAdd(&delta_n_kesei[4], -1); // Atomic addition

	float sdf_v3_plus; pworld_v = make_float3(pworld0.x , pworld0.y, pworld0.z+v_h);
	if (false == volume.interpolateSDF(pworld_v, sdf_v3_plus))ret = false;
//	else atomicAdd(&delta_n_kesei[5], 1); // Atomic addition
	float sdf_v3_minus; pworld_v = make_float3(pworld0.x, pworld0.y , pworld0.z-v_h);
	if (false == volume.interpolateSDF(pworld_v, sdf_v3_minus))ret=false;
//	else atomicAdd(&delta_n_kesei[5], -1); // Atomic addition
	if (ret == false)return false;
	output[0] = (sdf_w1_plus - sdf_w1_minus) / (2 * w_h);
	output[1] = (sdf_w2_plus - sdf_w2_minus) / (2 * w_h);
	output[2] = (sdf_w3_plus - sdf_w3_minus) / (2 * w_h);
	output[3] = (sdf_v1_plus - sdf_v1_minus) / (2 * v_h);
	output[4] = (sdf_v2_plus - sdf_v2_minus) / (2 * v_h);
	output[5] = (sdf_v3_plus - sdf_v3_minus) / (2 * v_h);
	output[6] = sdf0;
	return true;
}
__global__ void  computeSDFSolverbufKernel(const tsdfvolume volume, const DepthfMap2D depths, const CameraParams depth_camera_params, const Mat44 curTrans,DataMap2D<float> gbuf,
	const Mat44 plus_cur_w1, const Mat44  minus_cur_w1, const Mat44  plus_cur_w2, const Mat44 minus_cur_w2, const Mat44 plus_cur_w3, const Mat44  minus_cur_w3,
	float w_h, float v_h)
{
	const unsigned x = blockDim.x*blockIdx.x + threadIdx.x;
	const unsigned y = blockDim.y*blockIdx.y + threadIdx.y;
	__shared__ float smem[BLOCK_SIZE_2D_X*BLOCK_SIZE_2D_Y];
	unsigned tid = flattenedThreadId();
	smem[tid] = 0;
	float row[7] = { 0, 0, 0, 0, 0, 0, 0 };
	float d = depths.get_data(x, y);
	if (d == 0)
	{
	}
	else
	{
		float3	p=DepthCamera::depthToSkeleton(x, y, d,depth_camera_params);
		buildSDFSolverRows(volume, p, curTrans, plus_cur_w1, minus_cur_w1, plus_cur_w2, minus_cur_w2, plus_cur_w3, minus_cur_w3, w_h, v_h, row);
	}

	int blockId = blockIdx.x + gridDim.x * blockIdx.y;
	int shift = 0;
	for (int i = 0; i < 6; ++i)        //rows
	{
#pragma unroll
		for (int j = i; j < 7; ++j)          // cols + b
		{
			__syncthreads();
			smem[tid] = row[i] * row[j] ;
			__syncthreads();

			reduce(smem, BLOCK_SIZE_2D_X*BLOCK_SIZE_2D_Y);//ÇóºÍ

			if (tid == 0)
			{
				gbuf.set_data(blockId, shift++,smem[0]);
			}
		}
	}

}

void cudaCalSDFSolverParams(const CameraParams& depth_camera_params,const Mat44& cur_transform)
{
	tsdfvolume volume=CudaDeviceDataMan::instance()->_volume;
	DepthfMap2D depths=CudaDeviceDataMan::instance()->_trunced_depth;
	DataMap2D<float> buf_temp=CudaDeviceDataMan::instance()->_rigid_align_buf_temp_pyramid[0];
	DataMap1D<float> buf_reduced=CudaDeviceDataMan::instance()->_rigid_align_buf_reduced;


	float w_h = 0.001;
	float v_h = volume.size().x / volume.resolution().x;
	Mat44 plus_cur_w1, minus_cur_w1, plus_cur_w2, minus_cur_w2, plus_cur_w3, minus_cur_w3
		/*,plus_cur_v1, minus_cur_v1, plus_cur_v2, minus_cur_v2, plus_cur_v3, minus_cur_v3*/;

	Mat44 delta = Mat44::getIdentity();

	delta.m23 = -w_h; delta.m32 = w_h; plus_cur_w1 = delta*cur_transform;
	delta.m23 = w_h; delta.m32 = -w_h; minus_cur_w1 = delta*cur_transform;
	delta.m23 = 0; delta.m32 = 0;
	delta.m13 = w_h; delta.m31 = -w_h; plus_cur_w2 = delta*cur_transform;
	delta.m13 = -w_h; delta.m31 = w_h; minus_cur_w2 = delta*cur_transform;
	delta.m13 = 0; delta.m31 = 0;
	delta.m12 = -w_h; delta.m21 = w_h; plus_cur_w3 = delta*cur_transform;
	delta.m12 = w_h; delta.m21 = -w_h; minus_cur_w3 = delta*cur_transform;
	delta.m12 = 0; delta.m21 = 0;
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(depths.cols(), BLOCK_SIZE_2D_X), divUp(depths.rows(), BLOCK_SIZE_2D_Y));
	computeSDFSolverbufKernel << <gridSize, blockSize >> >(volume, depths, depth_camera_params, cur_transform, buf_temp, plus_cur_w1, minus_cur_w1, plus_cur_w2, minus_cur_w2, plus_cur_w3, minus_cur_w3, w_h, v_h);
	reduceGbufKernel << <buf_reduced.cols(), 512, 512 * sizeof(float) >> >(buf_temp.data(),buf_temp.cols(),buf_temp.rows(), buf_reduced.data(), 512);
}
