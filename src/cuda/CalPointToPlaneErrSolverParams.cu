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
__device__  bool findCorrs(int x,int y,
				const Point4fMap2D& input_vertices,
				const Vector4fMap2D& input_normals,
				const Point4fMap2D& target_vertices,
				const Vector4fMap2D& target_normals,
				const CameraParams& depth_camera_params,
				const Mat44 &cur_transform,
				const Mat44 &last_transform_inv,
				float dist_thres,
				float norm_sin_thres,
				float3 &n,
				float3 &p,
				float3 &q)
{
	const unsigned  cols=input_vertices.cols();
	const unsigned  rows=input_vertices.rows();
	float4 input_v_data = input_vertices.at(x, y);
	float4 input_n_data = input_normals.at(x, y);
	if (isZero(input_n_data) ) return false;
	float4 v_input_g = cur_transform* input_v_data ;
	float4 n_input_g =cur_transform* input_n_data;
	float4 v_cp = last_transform_inv*v_input_g;

	int2 screen_pos= DepthCamera::projectSkeletonToScreen(make_float3(v_cp.x,v_cp.y,v_cp.z),depth_camera_params);
	if (screen_pos.x < 0 || screen_pos.x >= cols || screen_pos.y < 0 || screen_pos.y >= rows)
	{
		return false;
	}
	float4 n_target_g = target_normals.at(screen_pos.x,screen_pos.y);
	if (isZero(n_target_g))return false;
	float4 v_target_g = target_vertices.at(screen_pos.x, screen_pos.y);

	float4 delta_g=v_target_g - v_input_g;
	float d = norm(make_float3(delta_g.x,delta_g.y,delta_g.z));
	float d_normal_sin = norm(cross(make_float3(n_target_g.x,n_target_g.y,n_target_g.z), make_float3(n_input_g.x,n_input_g.y,n_input_g.z)));
	if (d >dist_thres||d_normal_sin>norm_sin_thres)
	{
		return false;
	}
	p=make_float3(v_target_g.x,v_target_g.y,v_target_g.z);
	q=make_float3(v_input_g.x,v_input_g.y,v_input_g.z);
	n=make_float3(n_target_g.x,n_target_g.y,n_target_g.z);
	return true;
}
__global__ void computeGbufKernel(const Point4fMap2D input_v,
									const Vector4fMap2D input_n,
									const Point4fMap2D target_v,
									const Vector4fMap2D target_n,
									CudaMap2D<float> gbuf,
									const CameraParams depth_camera_params,
									const Mat44 cur_transform,
									const Mat44 last_transform_inv,
									float dist_thres,
									float norm_sin_thres)
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

	float row[7] = { 0, 0, 0, 0, 0, 0 ,0};
	if (x< cols&& y <rows && findCorrs(x,y,input_v,input_n,target_v,target_n,depth_camera_params,cur_transform,last_transform_inv,dist_thres,norm_sin_thres,n,p,q))
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
				gbuf.at(blockId, shift++)=smem[0];
			}
		}
	}

}

void cudaCalPointToPlaneErrSolverParams( unsigned pyramid_level,const Mat44& cur_transform,
		const Mat44& last_transform_inv,
		const CameraParams& depth_camera_params,
		float dist_thres,
		float norm_sin_thres)
{
	Point4fMap2D input_v=CudaDeviceDataMan::instance()->new_vertices_pyramid[pyramid_level];
	Vector4fMap2D input_n=CudaDeviceDataMan::instance()->new_normals_pyramid[pyramid_level];
	Point4fMap2D target_v=CudaDeviceDataMan::instance()->model_vertices_pyramid[pyramid_level];
	Point4fMap2D target_n=CudaDeviceDataMan::instance()->model_normals_pyramid[pyramid_level];
	CudaMap2D<float> icp_buf_temp=CudaDeviceDataMan::instance()->rigid_align_buf_temp_pyramid[pyramid_level];
	CudaMap1D<float> icp_buf_reduced=CudaDeviceDataMan::instance()->rigid_align_buf_reduced;
	const dim3 blockSize(BLOCK_SIZE_2D_X, BLOCK_SIZE_2D_Y);
	const dim3 gridSize(divUp(input_v.cols(), BLOCK_SIZE_2D_X), divUp(input_v.rows(), BLOCK_SIZE_2D_Y));
	computeGbufKernel << <gridSize, blockSize >> >(input_v, input_n, target_v,target_n, icp_buf_temp,
			depth_camera_params,cur_transform,last_transform_inv,dist_thres,norm_sin_thres);
	cudaDeviceSynchronize();
	reduceGbufKernel << <unsigned(icp_buf_reduced.count()), 512, 512 * sizeof(float) >> >(icp_buf_temp.ptr(), unsigned(icp_buf_temp.cols()),unsigned(icp_buf_temp.rows()),icp_buf_reduced.ptr(), 512);

}
