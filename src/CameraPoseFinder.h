/*
 * CameraPosFinder.h
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#ifndef CAMERAPOSEFINDER_H_
#define CAMERAPOSEFINDER_H_

#include"cuda/Mat.h"
#include"utils/cpu_include.h"
#include"AppParams.h"
#include"FrameData.h"
class CameraPoseFinder {
public:
	CameraPoseFinder():_inited(false){}
	virtual ~CameraPoseFinder()
	{
	}
	virtual bool init(const Mat44& reference_transform)
	{
		if(_inited)return false;
		_pose=reference_transform;
		if(false==initPoseFinder())return false;
		_inited=true;
		return _inited;
	}
	virtual bool findCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)
	{
		if(!_inited)return false;
		return estimateCameraPose(depth_frame,color_frame);
	}
	Mat44 getCameraPose()const {return _pose;}
	void setCameraPose(const Mat44& transform) { _pose=transform;}
protected:
	Mat44 _pose;
	virtual bool initPoseFinder()=0;
	virtual bool estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)=0;
private:

	bool _inited;
};
#endif /* CAMERAPOSFINDER_H_ */
