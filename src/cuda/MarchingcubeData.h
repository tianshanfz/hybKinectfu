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


/* UNTEST!!!*/
class MarchingcubeData
{
private:
	int* _ref_count;
	DeviceKind _dev_kind;
	Triangle* _triangles_ptr;
	unsigned*    _ptr_num_triangles;
	unsigned _max_triangles;
	void initData()
	{
		if(CPU==_dev_kind)
		{
			_ptr_num_triangles=new unsigned;
			_triangles_ptr=new Triangle[_max_triangles];
		}
		else if(GPU==_dev_kind)
		{
			cudaMalloc((void**)&_ptr_num_triangles,sizeof(unsigned));
			cudaMalloc((void**)&_triangles_ptr,sizeof(Triangle)*_max_triangles);
		}
		clearData();
	}
public:
	explicit MarchingcubeData(DeviceKind dev_kind):_dev_kind(dev_kind),_triangles_ptr(nullptr),_ptr_num_triangles(nullptr),_max_triangles(0),_ref_count(new int(1))
	{
	}
	MarchingcubeData(DeviceKind dev_kind,unsigned max_triangle_num):_dev_kind(dev_kind),_max_triangles(max_triangle_num),_ref_count(new int(1))
	{
		initData();
	}
	MarchingcubeData(const MarchingcubeData& from):_dev_kind(from._dev_kind),_triangles_ptr(from._triangles_ptr),_ptr_num_triangles(from._ptr_num_triangles),_max_triangles(from._max_triangles),_ref_count(from._ref_count)
	{
		(*_ref_count)++;
	}
	MarchingcubeData& operator=(const MarchingcubeData&) = delete;
	void resize(unsigned max_triangle_num)
	{
		destroyData();
		_max_triangles=max_triangle_num;
		initData();
	}
	virtual ~MarchingcubeData()
	{
		if(--(*_ref_count)==0)
		{
			destroyData();
			delete _ref_count;
			_ref_count=nullptr;
		}
	}
	__device__ __host__ Triangle& at(unsigned x)
	{
		return _triangles_ptr[x];
	}
	//__device__ __host__ Triangle* trianglePtr()const{return _triangles_ptr;}
	__device__ __host__ unsigned *triangleNumPtr()const{return _ptr_num_triangles;}
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
		}
		return ret;
	}
	void clearData()
	{
		if(_ptr_num_triangles!=nullptr)
		{
			if(CPU==_dev_kind)
			{
				memset(_ptr_num_triangles,0,sizeof(unsigned));
			}
			else if(GPU==_dev_kind)
			{
				cudaMemset((void*)_ptr_num_triangles,0,sizeof(unsigned));
			}
		}
		return;
	}
	void destroyData()
	{
		if(_ptr_num_triangles)
		{
			if(CPU==_dev_kind)	delete _ptr_num_triangles;
			else if(GPU==_dev_kind)cudaFree(_ptr_num_triangles);
			_ptr_num_triangles=nullptr;
		}
		if(_triangles_ptr)
		{
			if(CPU==_dev_kind)	delete []_triangles_ptr;
			else if(GPU==_dev_kind)cudaFree(_triangles_ptr);
			_triangles_ptr=nullptr;
		}
	}
	MarchingcubeData clone(DeviceKind target_dev_kind)const
	{
		MarchingcubeData ret(target_dev_kind,this->_max_triangles);
		cout<<triangleNums()<<endl;
		if(GPU==_dev_kind&&CPU==target_dev_kind)
		{
			cudaMemcpy(ret._triangles_ptr, _triangles_ptr, sizeof(Triangle)*triangleNums(), cudaMemcpyDeviceToHost);
			cudaMemcpy(ret._ptr_num_triangles, _ptr_num_triangles, sizeof(unsigned), cudaMemcpyDeviceToHost);
		}
		else if(GPU==_dev_kind&&GPU==target_dev_kind)
		{
			cudaMemcpy(ret._ptr_num_triangles, _ptr_num_triangles, sizeof(unsigned), cudaMemcpyDeviceToDevice);
			cudaMemcpy(ret._triangles_ptr,_triangles_ptr,  sizeof(Triangle)*triangleNums(), cudaMemcpyDeviceToDevice);
		}
		else if(CPU==_dev_kind&&CPU==target_dev_kind)
		{
			copy(_ptr_num_triangles, _ptr_num_triangles+1,ret._ptr_num_triangles );
			copy(_triangles_ptr,_triangles_ptr+triangleNums(),ret._triangles_ptr);
		}
		else
		{
			cudaMemcpy(ret._ptr_num_triangles, _ptr_num_triangles, sizeof(unsigned), cudaMemcpyHostToDevice);
			cudaMemcpy(ret._triangles_ptr,_triangles_ptr,  sizeof(Triangle)*triangleNums(), cudaMemcpyHostToDevice);
		}
		return ret;
	}

};
#endif /* MARCHINGCUBEDATA_H_ */
