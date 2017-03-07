/*
 * DataMap.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef DATAMAP_H_
#define DATAMAP_H_
//#include "cuda_include.h"
#include <memory>
#include <cuda_runtime_api.h>
#include<vector_functions.h>
#include<memory.h>
#include<iostream>
using namespace std;



typedef uchar3 Color3u;
typedef float4 Point4f;
typedef Point4f Vector4f;
typedef float3 Point3f;
typedef Point3f Vector3f;

using namespace std;
enum DeviceKind
{
	CPU,
	GPU
};
template <typename T>
class CudaMap1D
{
public:
	explicit CudaMap1D(DeviceKind dev_kind):_dev_kind(dev_kind),_elem_count(0),_ref_count(new int(1)),_ptr(nullptr)
	{

	}
	CudaMap1D(DeviceKind dev_kind,size_t n):_dev_kind(dev_kind),_elem_count(n),_ref_count(new int(1)),_ptr(nullptr)
	{
		initData();
	}
	CudaMap1D(const CudaMap1D& from):_elem_count(from._elem_count),_ref_count(from._ref_count),_ptr(from._ptr),_dev_kind(from._dev_kind)
	{
		(*_ref_count)++;
	}
	CudaMap1D& operator=(const CudaMap1D&) = delete;
	static CudaMap1D zeros(DeviceKind target_dev_kind,size_t n)
	{
		CudaMap1D ret(target_dev_kind,n);
		if(0==n)return ret;
		if(CPU==target_dev_kind)
			memset(ret._ptr,0,sizeof(T)*n);
		else if(GPU==target_dev_kind)
			cudaMemset(ret._ptr,0,sizeof(T)*n);
		return ret;
	}
	void resize(int elem_count)
	{
		destroyData();
		_elem_count=elem_count;
		initData();
	}
	__host__ __device__ const T* ptr()const
	{
		return _ptr;
	}
	__host__ __device__  T* ptr()
		{
			return _ptr;
		}
	__host__ __device__ T& at(unsigned x)
	{
		return _ptr[x];
	}
	__host__ __device__ T& at(unsigned x)const
	{
		return _ptr[x];
	}
	void copyDataFrom(const CudaMap1D& from)
	{//you must make sure enough space has been allocated before doing this!
		if(_dev_kind == CPU&&from._dev_kind == CPU)
		{
			copy(from._ptr, from._ptr+_elem_count, _ptr);
		}
		else if(_dev_kind == CPU&&from._dev_kind == GPU)
		{
			cudaMemcpy(_ptr, from._ptr, sizeof(T)*_elem_count, cudaMemcpyDeviceToHost);
		}
		else if(_dev_kind == GPU&&from._dev_kind == CPU)
		{
			cudaMemcpy(_ptr, from._ptr, sizeof(T)*_elem_count, cudaMemcpyHostToDevice);
		}
		else if(_dev_kind == GPU&&from._dev_kind == GPU)
		{
			cudaMemcpy(_ptr, from._ptr, sizeof(T)*_elem_count, cudaMemcpyDeviceToDevice);
		}
	}
	CudaMap1D clone(DeviceKind target_dev_kind)const
	{
		CudaMap1D ret(target_dev_kind,this->_elem_count);
	//	if(this->_dev_kind==UNSET||target_dev_kind==UNSET)return ret;
		if(target_dev_kind==CPU&&this->_dev_kind==CPU)
		{
			copy(this->_ptr,this->_ptr+_elem_count,ret._ptr);
		}
		else if(target_dev_kind==GPU&&this->_dev_kind==CPU)
		{
			cudaMemcpy(ret._ptr,_ptr, sizeof(T)*_elem_count, cudaMemcpyHostToDevice);
		}
		else if(target_dev_kind==CPU&&this->_dev_kind==GPU)
		{
			cudaMemcpy(ret._ptr,_ptr, sizeof(T)*_elem_count, cudaMemcpyDeviceToHost);
		}
		else
		{
			cudaMemcpy(ret._ptr,_ptr, sizeof(T)*_elem_count, cudaMemcpyDeviceToDevice);
		}
		return ret;
	}
	__host__ __device__ size_t count()const
	{
		return _elem_count;
	}
	void clearData()
	{//reset data to zeros
		if(_ptr!=nullptr)
		{
			if (_dev_kind == CPU)
			{
				memset(_ptr,0,sizeof(T)*_elem_count);
			}
			else if (_dev_kind == GPU)
			{
				cudaMemset(_ptr,0,sizeof(T)*_elem_count);
			}
		}
	}

	virtual ~CudaMap1D()
	{
		if(--(*_ref_count)==0)
		{
			destroyData();
			delete _ref_count;
			_ref_count=nullptr;
		}
	}
private:
	void destroyData()
	{
		if(_ptr!=nullptr)
		{
			if (_dev_kind == CPU)
			{
				delete[]_ptr;
			}
			else if (_dev_kind == GPU)
			{
				cudaFree(_ptr);
			}
			_ptr=nullptr;
			_elem_count=0;
		//	_dev_kind=UNSET;
		}
	}
	void initData()
	{//set _ptr according to _elem_count
		if(0==_elem_count)
		{
			_ptr=nullptr;
			return;
		}
		if(_dev_kind==CPU)
		{
			_ptr=new T[_elem_count]();
		}
		else if(_dev_kind==GPU)
		{
			cudaMalloc((void**)&_ptr,sizeof(T)*_elem_count);
		}
	}
	size_t  _elem_count;
	int* _ref_count;
	T*  _ptr;
	const DeviceKind _dev_kind;
};

template <typename T>
class CudaMap2D
{
public:
	explicit CudaMap2D(DeviceKind dev_kind):_dev_kind(dev_kind),_cols(0),_rows(0),_ref_count(new int(1)),_ptr(nullptr)
	{

	}
	CudaMap2D(DeviceKind dev_kind,size_t cols,size_t rows):_dev_kind(dev_kind),_cols(cols),_rows(rows),_ref_count(new int(1)),_ptr(nullptr)
	{
		initData();
	}

	CudaMap2D(const CudaMap2D& from):_cols(from._cols),_rows(from._rows),_ref_count(from._ref_count),_ptr(from._ptr),_dev_kind(from._dev_kind)
	{
		(*_ref_count)++;
	}
	CudaMap2D& operator=(const CudaMap2D&) = delete;
	static CudaMap2D zeros(DeviceKind target_dev_kind,size_t cols,size_t rows)
	{
		size_t elem_count=rows*cols;
		CudaMap2D ret(target_dev_kind,elem_count);
		if(0==elem_count)return ret;
		if(CPU==target_dev_kind)
			memset(ret._ptr,0,sizeof(T)*elem_count);
		else if(GPU==target_dev_kind)
			cudaMemset(ret._ptr,0,sizeof(T)*elem_count);
		return ret;
	}
	void resize(int cols,int rows)
	{
		destroyData();
		_cols=cols;
		_rows=rows;
		initData();
	}
	__host__ __device__ const T* ptr()const
	{
		return _ptr;
	}
	__host__ __device__  T* ptr()
	{
		return _ptr;
	}
	__host__ __device__ T& at(unsigned col,unsigned row)
	{
		return _ptr[row*_cols+col];
	}
	__host__ __device__ T& at(unsigned col,unsigned row)const
	{
		return _ptr[row*_cols+col];
	}
	void copyDataFrom(const CudaMap2D& from)
	{//you must make sure enough space has been allocated before doing this!
		size_t elem_count=from._rows*from._cols;
		if(_dev_kind == CPU&&from._dev_kind == CPU)
		{
			copy(from._ptr, from._ptr+elem_count, _ptr);
		}
		else if(_dev_kind == CPU&&from._dev_kind == GPU)
		{
			cudaMemcpy(_ptr, from._ptr, sizeof(T)*elem_count, cudaMemcpyDeviceToHost);
		}
		else if(_dev_kind == GPU&&from._dev_kind == CPU)
		{
			cudaMemcpy(_ptr, from._ptr, sizeof(T)*elem_count, cudaMemcpyHostToDevice);
		}
		else if(_dev_kind == GPU&&from._dev_kind == GPU)
		{
			cudaMemcpy(_ptr, from._ptr, sizeof(T)*elem_count, cudaMemcpyDeviceToDevice);
		}
	}
	CudaMap2D clone(DeviceKind target_dev_kind)const
	{
		CudaMap2D ret(target_dev_kind,this->_cols,this->_rows);
		size_t elem_count=_rows*_cols;
		ret.copyDataFrom(*this);
		return ret;
	}
	__host__ __device__ size_t cols()const
	{
		return _cols;
	}
	__host__ __device__ size_t rows()const
	{
		return _rows;
	}
	void clearData()
	{//reset data to zeros
		if(_ptr!=nullptr)
		{
			size_t elem_count=_rows*_cols;
			if (_dev_kind == CPU)
			{
				memset(_ptr,0,sizeof(T)*elem_count);
			}
			else if (_dev_kind == GPU)
			{
				cudaMemset(_ptr,0,sizeof(T)*elem_count);
			}
		}
	}

	virtual ~CudaMap2D()
	{
		if(--(*_ref_count)==0)
		{
			destroyData();
			delete _ref_count;
			_ref_count=nullptr;
		}
	}

private:
	void destroyData()
	{
		if(_ptr!=nullptr)
		{
			if (_dev_kind == CPU)
			{
				delete[]_ptr;
			}
			else if (_dev_kind == GPU)
			{
				cudaFree(_ptr);
			}
			_ptr=nullptr;
			_cols=_rows=0;
		//	_dev_kind=UNSET;
		}
	}
	void initData()
	{
		size_t elem_count=_cols*_rows;
		if(0==elem_count)
		{
			_ptr=nullptr;
			return;
		}
		if(_dev_kind==CPU)
		{
			_ptr=new T[elem_count]();
		}
		else if(_dev_kind==GPU)
		{
			cudaMalloc((void**)&_ptr,sizeof(T)*elem_count);
		}
	}
	size_t  _cols;
	size_t _rows;
	int* _ref_count;
	T*  _ptr;
	const DeviceKind _dev_kind;
};

typedef CudaMap2D<float4> ColorfMap2D;
typedef CudaMap2D<float> DepthfMap2D;
typedef CudaMap2D<Point4f>  Point4fMap2D;
typedef CudaMap2D<Vector4f> Vector4fMap2D;
typedef CudaMap2D<Color3u> Color3uMap2D;

#endif /* DATAMAP_H_ */
