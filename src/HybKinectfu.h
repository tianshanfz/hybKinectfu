/*
 * HybKinectfu.h
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#ifndef HYBKINECTFU_H_
#define HYBKINECTFU_H_
#include"utils/cpu_include.h"
#include"CameraPoseFinder.h"
#include"TrajectoryRecorder.h"
#include"FrameData.h"
class HybKinectfu {
public:
	HybKinectfu();
	virtual ~HybKinectfu();
	bool init();
	bool processNewFrame(const DepthFrameData& depth_frame,const ColorFrameData& rgb_frame);
private:
	void copyFrameToGPU(const DepthFrameData& depth_frame,const ColorFrameData& color_frame);
	CameraPoseFinder* _camera_pose_finder;
	//SRKeyframeMan* _sr_frame_man;
	TrajectoryRecorder* _camera_pose_recorder;
	bool _inited;

};

#endif /* HYBKINECTFU_H_ */
