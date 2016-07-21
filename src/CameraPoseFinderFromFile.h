/*
 * CameraPosFinderFromFile.h
 *
 *  Created on: May 30, 2016
 *      Author: hanyinbo
 */

#ifndef CameraPoseFinderFromFile_H_
#define CameraPoseFinderFromFile_H_
#include "CameraPoseFinder.h"

class CameraPoseFinderFromFile:public CameraPoseFinder {
public:
	CameraPoseFinderFromFile();
	virtual ~CameraPoseFinderFromFile();

protected:
	virtual bool initPoseFinder();
	virtual bool estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame);

private:
	bool parseFrameFromFile(double timestamp,float3& t,float4& q,double &res_timestemp);
	std::ifstream _trajfile_src;
	Mat44 _refer_transform;


};

#endif /* CameraPoseFinderFromFile_H_ */
