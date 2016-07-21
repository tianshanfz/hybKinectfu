/*
 * CameraPoseFinderICP.h
 *
 *  Created on: May 29, 2016
 *      Author: hanyinbo
 */

#ifndef CameraPoseFinderICP_H_
#define CameraPoseFinderICP_H_
#include "CameraPoseFinder.h"
#include "AppParams.h"
#include <Eigen/Eigen>
class CameraPoseFinderICP:public CameraPoseFinder {
public:
	CameraPoseFinderICP(){}
	virtual ~CameraPoseFinderICP(){}
protected:
	virtual bool initPoseFinder();
	virtual bool estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame);
private:
	bool vector6ToTransformMatrix(const Eigen::Matrix<float, 6, 1, 0, 6, 1>& x,  Eigen::Matrix4f& output);
	void findCorresSet(const unsigned level,const Mat44& cur_transform,const Mat44& last_transform_inv);
	bool minimizePointToPlaneErrFuncHost(unsigned level,Eigen::Matrix<float, 6, 1> &six_dof);
	bool minimizePointToPlaneErrFunc(unsigned level,Eigen::Matrix<float, 6, 1> &six_dof);
	vector<int> _iter_nums;
	vector<CameraParams> _camera_params_pyramid;
};

#endif /* CameraPoseFinderICP_H_ */
