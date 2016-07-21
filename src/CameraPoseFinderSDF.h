/*
 * CameraPoseFinderSDF.h
 *
 *  Created on: Jun 2, 2016
 *      Author: hanyinbo
 */

#ifndef CameraPoseFinderSDF_H_
#define CameraPoseFinderSDF_H_
#include "CameraPoseFinder.h"
#include <Eigen/Eigen>
class CameraPoseFinderSDF:public CameraPoseFinder {
public:
	CameraPoseFinderSDF();
	virtual ~CameraPoseFinderSDF();
protected:
	virtual bool initPoseFinder();
	virtual bool estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame);
	bool vector6ToTransformMatrix(Eigen::Matrix<float, 6, 1, 0, 6, 1>& x,  Eigen::Matrix4f& output);
};

#endif /* CameraPoseFinderSDF_H_ */
