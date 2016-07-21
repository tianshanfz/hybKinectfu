/*
 * MarchingcubeData.h
 *
 *  Created on: Jun 7, 2016
 *      Author: hanyinbo
 */

#ifndef MARCHINGCUBEDATA_H_
#define MARCHINGCUBEDATA_H_



#include"cuda_include.h"
#include"DataMap.h"
typedef struct Vertex
{
	float3 pos;
	float3 color;
}Vertex;
typedef struct Triangle
{
	Vertex v0;
	Vertex v1;
	Vertex v2;
}Triangle;
class MarchingcubeData
{
private:
	DeviceKind _dev_kind=UNSET;
	Triangle* _triangles=nullptr;
	unsigned*    _ptr_num_triangles=nullptr;
	unsigned _max_triangles=0;
public:
	__device__ __host__ Triangle* triangleData()const{return _triangles;}
	__device__ __host__ unsigned *ptrNum()const{return _ptr_num_triangles;}
	__host__ __device__ unsigned maxTriangles()const{return _max_triangles;}
	unsigned triangleNums()const
	{
		unsigned ret=0;
		if(CPU==_dev_kind)
		{
			ret= *_ptr_num_triangles;
		}
		else if(GPU==_dev_kind)
		{
			cudaMemcpy(&ret, _ptr_num_triangles, sizeof(unsigned), cudaMemcpyDeviceToHost);
			return ret;
		}
		return ret;
	}
	void init(const DeviceKind& dev_kind)
	{
		_dev_kind=dev_kind;
		_max_triangles=0;
		if(CPU==_dev_kind)
		{
			_ptr_num_triangles=new unsigned;
		}
		else if(GPU==_dev_kind)
		{
			cudaMalloc((void**)&_ptr_num_triangles,sizeof(unsigned));
		}
		reset();
	}
	void init(unsigned max_triangles,const DeviceKind& dev_kind)
	{
		_dev_kind=dev_kind;
		_max_triangles=max_triangles;
		if(CPU==_dev_kind)
		{
			_ptr_num_triangles=new unsigned;
			_triangles=new Triangle[max_triangles];
		}
		else if(GPU==_dev_kind)
		{
			cudaMalloc((void**)&_ptr_num_triangles,sizeof(unsigned));
			cudaMalloc((void**)&_triangles,sizeof(Triangle)*max_triangles);
		}
		else
		{
			return ;
		}
		return reset();
	}
	void reset()
	{
		if(CPU==_dev_kind)
		{
			memset(_ptr_num_triangles,0,sizeof(unsigned));
		}
		else if(GPU==_dev_kind)
		{
			cudaMemset((void*)_ptr_num_triangles,0,sizeof(unsigned));
		}
		return;
	}
	void destroy()
	{
		if(_ptr_num_triangles)
		{
			if(CPU==_dev_kind)	delete _ptr_num_triangles;
			else if(GPU==_dev_kind)cudaFree(_ptr_num_triangles);
			_ptr_num_triangles=nullptr;
		}
		if(_triangles)
		{
			if(CPU==_dev_kind)	delete []_triangles;
			else if(GPU==_dev_kind)cudaFree(_triangles);
			_triangles=nullptr;
		}
	}
	void copyFrom(const MarchingcubeData& from_data)
	{
		if(from_data._dev_kind!=UNSET&&_dev_kind!=UNSET)
		{
			destroy();
			init(from_data._max_triangles, _dev_kind);
		}
		if(CPU==_dev_kind&&GPU==from_data._dev_kind)
		{
			cudaMemcpy(_triangles, from_data._triangles, sizeof(Triangle)*from_data.triangleNums(), cudaMemcpyDeviceToHost);
			cudaMemcpy(_ptr_num_triangles, from_data._ptr_num_triangles, sizeof(unsigned), cudaMemcpyDeviceToHost);
		}
		else if(GPU==_dev_kind&&GPU==from_data._dev_kind)
		{
			cudaMemcpy(_ptr_num_triangles, from_data._ptr_num_triangles, sizeof(unsigned), cudaMemcpyDeviceToDevice);
			cudaMemcpy(_triangles, from_data._triangles, sizeof(Triangle)*from_data.triangleNums(), cudaMemcpyDeviceToDevice);
		}
		else if(CPU==_dev_kind&&CPU==from_data._dev_kind)
		{
			memcpy(_ptr_num_triangles, from_data._ptr_num_triangles, sizeof(unsigned));
			memcpy(_triangles,from_data._triangles, sizeof(Triangle)*from_data.triangleNums());
		}
		else
		{
			cudaMemcpy(_ptr_num_triangles, from_data._ptr_num_triangles, sizeof(unsigned), cudaMemcpyHostToDevice);
			cudaMemcpy(_triangles, from_data._triangles, sizeof(Triangle)*from_data.triangleNums(), cudaMemcpyHostToDevice);
		}
	}

};
#endif /* MARCHINGCUBEDATA_H_ */
