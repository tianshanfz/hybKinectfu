/*
 * DepthCamera.h
 *
 *  Created on: May 28, 2016
 *      Author: hanyinbo
 */

#ifndef DEPTHCAMERA_H_
#define DEPTHCAMERA_H_




#include"../AppParams.h"
class DepthCamera
{
public:

	__device__   __host__ static float3 depthToSkeleton(unsigned int ux, unsigned int uy, float depth,const CameraParams& params)
	{
		float z = depth;
		float cx =params.cx;
		float cy = params.cy;
		float vx1 = z *(ux - cx)/params.fx;
		float vy1 = z *(uy-cy) /params.fy;
		float vz = z;

		return make_float3(vx1, vy1, vz);
	}
	__device__   __host__ static float2 projectSkeletonToScreenfloat(const float3& v,const CameraParams& params)
	{
		float cx = params.cx;//ÖÐÐÄ×ø±ê
		float cy = params.cy;

		float retx1 = v.x *params.fx/ v.z + cx;
		float rety1 = v.y*params.fy / v.z  + cy;
		return make_float2(retx1, rety1);
	}
	__device__   __host__ static int2 projectSkeletonToScreen(const float3& v,const CameraParams& params)
	{
		float2 pImage = projectSkeletonToScreenfloat(v,params);
		return make_int2((int)(pImage.x+0.5),(int)(pImage.y+0.5));
	}


};




#endif /* DEPTHCAMERA_H_ */
