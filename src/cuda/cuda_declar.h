/*
 * cuda_declar.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef CUDA_DECLAR_H_
#define CUDA_DECLAR_H_


#ifndef BLOCK_SIZE_2D_X
#define BLOCK_SIZE_2D_X 16
#endif
#ifndef BLOCK_SIZE_2D_Y
#define BLOCK_SIZE_2D_Y 16
#endif
#ifndef BLOCK_SIZE_1D
#define BLOCK_SIZE_1D 512
#endif
#include<cmath>
#include <cuda_runtime_api.h>
#include<vector_functions.h>
#include"DataMap.h"




__host__ __device__ __forceinline__ bool isZero(const float3& v)
{
	return v.x==0&&v.y==0&&v.z==0;
}
__host__ __device__ __forceinline__ bool isZero(const uchar3& v)
{
	return v.x==0&&v.y==0&&v.z==0;
}
__host__ __device__ __forceinline__ bool isZero(const float4& v)
{
	return v.x==0&&v.y==0&&v.z==0&&v.w==0;
}
__host__ __device__ __forceinline__ float3& operator+=(float3& v1, const float3& v2)
{
	v1.x += v2.x;  	v1.y += v2.y;	v1.z+= v2.z; return v1;
}
__host__ __device__ __forceinline__ float3 operator*(const float3& v1, const float& v)
{
	return make_float3(v1.x * v, v1.y * v, v1.z * v);
}
__host__ __device__ __forceinline__ float4 operator*(const float4& v1, const float& v)
{
	return make_float4(v1.x * v, v1.y * v, v1.z * v,v1.w * v);
}
__host__ __device__ __forceinline__ float3 operator+(const float3& v1, const float3& v2)
{
	return make_float3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}
__host__ __device__ __forceinline__ int3 operator+(const int3& v1, const int3& v2)
{
	return make_int3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}
__host__ __device__ __forceinline__ float4 operator+(const float4& v1, const float4& v2)
{
	return make_float4(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z,v1.w + v2.w);
}
__host__ __device__ __forceinline__ float3 operator-(const float3 &a, const float3 &b) {

  return make_float3(a.x-b.x, a.y-b.y, a.z-b.z);
}
__host__ __device__ __forceinline__ float4 operator-(const float4& v1, const float4& v2)
{
	return make_float4(v1.x - v2.x, v1.y - v2.y, v1.z -v2.z,v1.w - v2.w);
}
__host__ __device__  __forceinline__ float dot(const float3& v1, const float3& v2)
{
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}
__host__ __device__  __forceinline__ float3 cross(const float3& v1, const float3& v2)
{
	float3 ret;
	ret.x = v1.y*v2.z - v1.z*v2.y;
	ret.y = v1.z*v2.x - v1.x*v2.z;
	ret.z = v1.x*v2.y - v1.y*v2.x;
	return ret;
}
__host__ __device__ __forceinline__ float norm(const float3& v)
{
	return sqrt(dot(v, v));
}
__host__ __device__ __forceinline__ float3 normalize(const float3& v)
{
	 float len=norm(v);
	 if(len<1e-8)return make_float3(0,0,0);
	 return v*(1.0/len);
}
__host__ __device__ __forceinline__ int sign(float val) {
	return (0 < val) - (val < 0);
}
__device__ __host__ __forceinline__ bool operator==(const int3& a,const int3& b)
{
	return a.x == b.x&&a.y == b.y&&a.z == b.z;
}
__forceinline__ int divUp(int n, int div)
{
	return (n + div - 1) / div;
}



#endif /* CUDA_DECLAR_H_ */
