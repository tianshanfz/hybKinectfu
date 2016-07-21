/*
 * DataMap.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef DATAMAP_H_
#define DATAMAP_H_
#include "cuda_include.h"
#include <memory.h>
#include<iostream>
using namespace std;
typedef uchar3 Color3u;
typedef float4 Point4f;
typedef Point4f Vector4f;
typedef float3 Point3f;
typedef Point3f Vector3f;

enum DeviceKind
{
	UNSET, CPU, GPU
};
template<typename T>
class DataMap1D
{
private:
	unsigned _cols;
	T* _data = nullptr;
	DeviceKind _dev_kind = UNSET;
public:
	__device__ __host__  unsigned cols()const{return _cols;}
	__device__ __host__ const T* data()const	{return _data;}
	__device__ __host__  T* data(){return _data;}
	__device__ __host__ void set_data(unsigned x, const T& value)
	{
		//		assert(!(data == NULL || x < 0 || x >= cols || y < 0 || y >= rows));
		_data[x] = value;
	}
	__device__ __host__ const T get_data(unsigned x)const
	{
		//	assert(data != NULL);
		//		assert(!( x < 0 || x >= cols || y < 0 || y >= rows));
		return _data[x];
	}
	DeviceKind type()const
	{
		return _dev_kind;
	}
	void setZero()
	{
		if (_dev_kind == GPU)
		{
			cudaMemset(_data, 0, sizeof(T)*_cols);
		}
		else if (_dev_kind == CPU)
		{
			memset(_data, 0, sizeof(T)*_cols);
		}
		else
		{
			return;
		}
	}
	bool create_gpu(unsigned cols)
	{
		if (_dev_kind != UNSET)
		{
			return false;
		}
		_dev_kind = GPU;
		this->_cols = cols;
		cudaMalloc((void**)&_data, sizeof(T)*cols);
		return true;
	}
	bool create_gpu(const DataMap1D<T>& t)
	{
		if (_dev_kind != UNSET || t._dev_kind == UNSET)
		{
			return false;
		}
		_dev_kind = GPU;
		this->_cols = t._cols;
		cudaMalloc((void**)&_data, sizeof(T)*_cols);
		if (t._dev_kind == CPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols, cudaMemcpyHostToDevice);
		}
		else/* if (t._dev_kind ==GPU)*/
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols, cudaMemcpyDeviceToDevice);
		}
		return true;
	}
	bool create_cpu(unsigned cols)
	{
		if (_dev_kind != UNSET)
		{
			return false;
		}
		_dev_kind = CPU;
		this->_cols = cols;
		_data = new T[cols];
		return 0;
	}
	bool copyFrom(const DataMap1D<T>& t)
	{
		if(_cols != t._cols)return false;
		if (_dev_kind == CPU&&t._dev_kind == CPU)
		{
			memcpy(_data, t._data, sizeof(T)*_cols);
		}
		else	if (_dev_kind == CPU&&t._dev_kind == GPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols, cudaMemcpyDeviceToHost);
		}
		else if (_dev_kind == GPU&&t._dev_kind == CPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols, cudaMemcpyHostToDevice);
		}
		else if (_dev_kind == GPU&&t._dev_kind == GPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols, cudaMemcpyDeviceToDevice);
		}
		return true;
	}
	bool create_cpu(const DataMap1D<T>& t)
	{
		if (_dev_kind != UNSET || t._dev_kind == UNSET)
		{
			return false;
		}
		_dev_kind = CPU;
		this->_cols = t._cols;
		_data = new T[_cols];
		if (t._dev_kind == CPU)
		{
			memcpy(_data, t._data, sizeof(T)*_cols);
		}
		else/* if (t._dev_kind ==GPU)*/
		{

			cudaMemcpy(_data, t._data, sizeof(T)*_cols, cudaMemcpyDeviceToHost);
		}
		return true;
	}
	void destroy()
	{
		if (_data)
		{
			if (_dev_kind == CPU)
			{
				delete[]_data;
				_data=nullptr;
			}
			else if (_dev_kind == GPU)
			{
				cudaFree(_data);
				_data=nullptr;
			}
			_dev_kind = UNSET;
		}
	}
};
template<typename T>
struct DataMap2D
{
private:
	unsigned _cols;
	unsigned _rows;
	T* _data=nullptr;
	DeviceKind _dev_kind = UNSET;
public:

	__device__ __host__  unsigned cols()const{return _cols;}
	__device__ __host__  unsigned rows()const{return _rows;}
	__device__ __host__ const T* data()const{return _data;}
	__device__ __host__  T* data(){return _data;}
	__device__ __host__ void set_data(unsigned x, unsigned y, const T& value)
	{
//		assert(!(data == NULL || x < 0 || x >= cols || y < 0 || y >= rows));
		_data[x+y*_cols] = value;
	}
	__device__ __host__ const T get_data(unsigned x, unsigned  y)const
	{
	//	assert(data != NULL);
//		assert(!( x < 0 || x >= cols || y < 0 || y >= rows));
		return _data[x + y*_cols];
	}
	void setZero()
	{
		if (_dev_kind == CPU)
		{
			memset(_data, 0, sizeof(T)*_cols*_rows);
		}
		else if (_dev_kind == GPU)
		{
			cudaMemset(_data, 0, sizeof(T)*_cols*_rows);
		}
	}
	DeviceKind type()const
	{
		return _dev_kind;
	}
	bool create_gpu(unsigned cols, unsigned rows)
	{
		if (_dev_kind != UNSET)
		{
			return false;
		}
		_dev_kind = GPU;
		this->_cols = cols;
		this->_rows = rows;
		cudaMalloc((void**)&_data, sizeof(T)*cols*rows);
		return true;
	}
	bool create_gpu(const DataMap2D<T>& t)
	{
		if (_dev_kind != UNSET||t._dev_kind==UNSET)
		{
			return false;
		}
		_dev_kind = GPU;
		this->_cols = t._cols;
		this->_rows = t._rows;
		cudaMalloc((void**)&_data, sizeof(T)*_cols*_rows);
		if (t._dev_kind == CPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols*_rows, cudaMemcpyHostToDevice);
		}
		else/* if (t._dev_kind ==GPU)*/
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols*_rows, cudaMemcpyDeviceToDevice);
		}
		return true;
	}
	bool create_cpu(unsigned cols, unsigned rows)
	{
		if (_dev_kind != UNSET)
		{
			return false;
		}
		_dev_kind = CPU;
		this->_rows = rows;
		this->_cols = cols;
		_data = new T[cols*rows];
		return true;
	}
	bool copyFrom(const DataMap2D<T>& t)
	{
		if(!(_cols == t._cols&&_rows == t._rows))return false;
		if (_dev_kind == CPU&&t._dev_kind == CPU)
		{
			memcpy(_data, t._data, sizeof(T)*_cols*_rows);
		}
		else	if (_dev_kind == CPU&&t._dev_kind == GPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols*_rows,cudaMemcpyDeviceToHost);
		}
		else if (_dev_kind == GPU&&t._dev_kind == CPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols*_rows, cudaMemcpyHostToDevice);
		}
		else if (_dev_kind == GPU&&t._dev_kind == GPU)
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols*_rows, cudaMemcpyDeviceToDevice);
		}
		return true;
	}
	bool create_cpu(const DataMap2D<T>& t)
	{
		if (_dev_kind != UNSET || t._dev_kind == UNSET)
		{
			return false;
		}
		_dev_kind = CPU;
		this->_cols = t._cols;
		this->_rows = t._rows;
		_data = new T[_cols*_rows];
		if (t._dev_kind == CPU)
		{
			memcpy(_data, t._data, sizeof(T)*_cols*_rows);
		}
		else/* if (t._dev_kind ==GPU)*/
		{
			cudaMemcpy(_data, t._data, sizeof(T)*_cols*_rows, cudaMemcpyDeviceToHost);
		}
		return 0;
	}

	void destroy()
	{
		if (_data)
		{
			if (_dev_kind == CPU)
			{
				delete[]_data;
				_data=nullptr;
			}
			else if (_dev_kind == GPU)
			{
				cudaFree(_data);
				_data=nullptr;
			}
			_dev_kind = UNSET;
		}
	}
};
typedef DataMap2D<float4> ColorfMap2D;
typedef DataMap2D<float> DepthfMap2D;
typedef DataMap2D<Point4f>  Point4fMap2D;
typedef DataMap2D<Vector4f> Vector4fMap2D;
typedef DataMap2D<Color3u> Color3uMap2D;

#endif /* DATAMAP_H_ */
