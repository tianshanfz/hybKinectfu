/*
 * CameraPosFinderSIFT.h
 *
 *  Created on: Jun 1, 2016
 *      Author: hanyinbo
 */
#if 1
#ifndef CAMERAPOSEFINDERSIFT_H_
#define CAMERAPOSFINDERSIFT_H_
#include"../CameraPoseFinder.h"
class CameraPoseFinderSIFT :public CameraPoseFinder{
public:
	CameraPoseFinderSIFT();
	virtual ~CameraPoseFinderSIFT();
protected:
	virtual bool initPoseFinder();
	virtual bool estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame);
};

#endif /* CAMERAPOSFINDERSIFT_H_ */
#endif
