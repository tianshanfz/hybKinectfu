/*
 * tsdfVolume.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef TSDFVOLUME_H_
#define TSDFVOLUME_H_
#include"DataMap.h"
#include"cuda_include.h"
#include"cuda_declar.h"
#include"../AppParams.h"
struct Voxel
{
	float tsdf;
	float weight;
	uchar3 color;
};
class tsdfvolume
{
public:
	tsdfvolume(DeviceKind dev_kind):_data(dev_kind)
	{

	}
	__device__ __host__ float3 size()const{ return _size; }
	__device__ __host__ int3 resolution()const{ return _resolution; }
	bool init(const tsdfVolumeParams& params)
	{
		_size = make_float3(params.fVolumeMeterSize, params.fVolumeMeterSize, params.fVolumeMeterSize);
		_resolution = make_int3(params.nResolution, params.nResolution, params.nResolution);
		_max_weight=params.fWeightMax;
		_data.resize(_resolution.x, _resolution.y*_resolution.z);
		_data.clearData();
		return true;
	}
	__device__ __host__ float3 voxelPosToWorld(const int3& pv)const
	{
		float3 coo = make_float3(pv.x, pv.y, pv.z);
		coo.x += 0.5f;         //shift to cell center;
		coo.y += 0.5f;         //shift to cell center;
		coo.z += 0.5f;         //shift to cell center;
		coo.x *= _size.x / _resolution.x;
		coo.y *= _size.y / _resolution.y;
		coo.z *= _size.z / _resolution.z;

		return coo;
	}
	__device__ __host__ int3 worldPosToVoxel(const float3& worldPos)const
	{
		int vx = (int)(worldPos.x *_resolution.x / _size.x);
		int vy = (int)(worldPos.y*_resolution.y / _size.y);
		int vz = (int)(worldPos.z*_resolution.z / _size.z);
		return make_int3(vx, vy, vz);
	}
	__device__ __host__ void updateVoxel(int x, int y, int z, float tsdf, float weight,const uchar3& color,float weight_color)
	{
		int row = z*_resolution.y + y;
		int col = x;
		float oldtsdf = _data.at(col,row).tsdf;
		float oldweight = _data.at(col,row).weight;
		float3 oldcolor = make_float3(_data.at(col, row).color.x, _data.at(col, row).color.y, _data.at(col, row).color.z);
		Voxel newvoxel;
		newvoxel.weight = fminf(oldweight + weight, _max_weight);
		newvoxel.tsdf = (oldtsdf*oldweight + tsdf*weight) / (oldweight + weight);

		newvoxel.color.x = fminf(255.0,(oldcolor.x*oldweight + color.x*weight_color) / (oldweight + weight_color));
		newvoxel.color.y=  fminf(255.0,(oldcolor.y*oldweight + color.y*weight_color) / (oldweight + weight_color));
		newvoxel.color.z = fminf(255.0,(oldcolor.z*oldweight + color.z*weight_color) / (oldweight + weight_color));

		_data.at(col,row)= newvoxel;
		return ;
	}
	__device__ __host__  Voxel getVoxel(int3 voxelPos)const
	{
		int row = voxelPos.z*_resolution.y + voxelPos.y;
		int col = voxelPos.x;
		return  _data.at(col, row);
	}
	__device__ __host__  bool getVoxel(const float3& worldPos, Voxel &v)const
	{
		int vx = (int)(worldPos.x*_resolution.x/_size.x);
		int vy = (int)(worldPos.y*_resolution.y/_size.y);
		int vz = (int)(worldPos.z*_resolution.z/_size.z);
/*		if (vx < 0 || vx >= _resolution.x ||  vy <= 0|| vy >= _resolution.y || vz < 0 || vz >= _resolution.z)
		{
			return false;
		}*/
		vx = max(0, min(vx, _resolution.x - 1));
		vy = max(0, min(vy, _resolution.y - 1));
		vz = max(0, min(vz, _resolution.z- 1));
		int row = vz*_resolution.y + vy;
		int col = vx;
		v= _data.at(col, row);
		return true;
	}
	__device__ __host__ bool interpolateSDF(const float3& pos, float& dist)const
	{
		int3 g;
		float a,b,c;
		if(false==getInterpolateParams(pos,g,a,b,c))return false;
		Voxel v000 = getVoxel(g); if (v000.weight == 0){ return false; }
		Voxel v001 = getVoxel(make_int3(g.x, g.y, g.z + 1)); if (v001.weight == 0){ return false; }
		Voxel v010 = getVoxel(make_int3(g.x, g.y + 1, g.z)); if (v010.weight == 0){ return false; }
		Voxel v011 = getVoxel(make_int3(g.x, g.y + 1, g.z + 1));if (v011.weight == 0){ return false; }
		Voxel v100 = getVoxel(make_int3(g.x + 1, g.y, g.z));if (v100.weight == 0){ return false; }
		Voxel v101 = getVoxel(make_int3(g.x + 1, g.y, g.z + 1)); if (v101.weight == 0){ return false; }
		Voxel v110 = getVoxel(make_int3(g.x + 1, g.y + 1, g.z)); if (v110.weight == 0){ return false; }
		Voxel v111 = getVoxel(make_int3(g.x + 1, g.y + 1, g.z + 1)); if (v111.weight == 0){ return false; }
		dist = v000.tsdf * (1 - a) * (1 - b) * (1 - c) +
			v001.tsdf * (1 - a) * (1 - b) * c +
			v010.tsdf * (1 - a) * b * (1 - c) +
			v011.tsdf * (1 - a) * b * c +
			v100.tsdf* a * (1 - b) * (1 - c) +
			v101.tsdf * a * (1 - b) * c +
			v110.tsdf * a * b * (1 - c) +
			v111.tsdf* a * b * c;
		return true;
	}

	__device__ __host__
		bool interpolateColor(const float3& pos, uchar3& color)const  {
		int3 g;
		float a,b,c;
		if(false==getInterpolateParams(pos,g,a,b,c)) return false;

		Voxel v000 = getVoxel(g); if (v000.weight == 0){ return false; }
		Voxel v001 = getVoxel(make_int3(g.x, g.y, g.z + 1)); if (v001.weight == 0){ return false; }
		Voxel v010 = getVoxel(make_int3(g.x, g.y + 1, g.z)); if (v010.weight == 0){ return false; }
		Voxel v011 = getVoxel(make_int3(g.x, g.y + 1, g.z + 1));if (v011.weight == 0){ return false; }
		Voxel v100 = getVoxel(make_int3(g.x + 1, g.y, g.z));if (v100.weight == 0){ return false; }
		Voxel v101 = getVoxel(make_int3(g.x + 1, g.y, g.z + 1)); if (v101.weight == 0){ return false; }
		Voxel v110 = getVoxel(make_int3(g.x + 1, g.y + 1, g.z)); if (v110.weight == 0){ return false; }
		Voxel v111 = getVoxel(make_int3(g.x + 1, g.y + 1, g.z + 1)); if (v111.weight == 0){ return false; }
		float3 colorRes =
			make_float3(v000.color.x,v000.color.y,v000.color.z) * (1 - a) * (1 - b) * (1 - c) +
			make_float3(v001.color.x, v001.color.y, v001.color.z)  * (1 - a) * (1 - b) * c +
			make_float3(v010.color.x, v010.color.y, v010.color.z)  * (1 - a) * b * (1 - c) +
			make_float3(v011.color.x, v011.color.y, v011.color.z)  * (1 - a) * b * c +
			make_float3(v100.color.x, v100.color.y, v100.color.z)  * a * (1 - b) * (1 - c) +
			make_float3(v101.color.x, v101.color.y, v101.color.z) * a * (1 - b) * c +
			make_float3(v110.color.x, v110.color.y, v110.color.z) * a * b * (1 - c) +
			make_float3(v111.color.x, v111.color.y, v111.color.z) * a * b * c;

		color = make_uchar3(colorRes.x, colorRes.y, colorRes.z);//v.color;
		return true;
	}
	//int writeVolumeToFile(int frameid);
protected:
	__host__ __device__ bool getInterpolateParams(const float3& pos,int3& base,float &a,float &b,float &c)const
	{
		int3  g = worldPosToVoxel(pos);
		if (g.x <= 0 || g.x >= resolution().x - 1)return false;
		if (g.y <= 0 || g.y >= resolution().y - 1)return false;
		if (g.z <= 0 || g.z >= resolution().z - 1)return false;

		float3 cell_size;
		cell_size.x =size().x / resolution().x;
		cell_size.y =size().y / resolution().y;
		cell_size.z =size().z / resolution().z;
		float vx = (g.x + 0.5f) * cell_size.x;//center pos
		float vy = (g.y + 0.5f) * cell_size.y;
		float vz = (g.z + 0.5f) * cell_size.z;

		g.x = (pos.x < vx) ? (g.x - 1) : g.x;
		g.y = (pos.y < vy) ? (g.y - 1) : g.y;
		g.z = (pos.z < vz) ? (g.z - 1) : g.z;
		base=g;
		a = (pos.x - (g.x + 0.5f) * cell_size.x) / cell_size.x;
		b = (pos.y - (g.y + 0.5f) * cell_size.y) / cell_size.y;
		c = (pos.z - (g.z + 0.5f) * cell_size.z) / cell_size.z;
		return true;
	}
	float3 _size;//meter
	int3 _resolution;
	float _max_weight;
	CudaMap2D<Voxel> _data;

};
#endif
