/*
 * TrajectoryRecorder.h
 *
 *  Created on: May 31, 2016
 *      Author: hanyinbo
 */

#ifndef TRAJECTORYRECORDER_H_
#define TRAJECTORYRECORDER_H_
#include"utils/cpu_include.h"
#include"cuda/Mat.h"
class TrajectoryRecorder {
public:
	TrajectoryRecorder(const string& record_filename);
	virtual ~TrajectoryRecorder();
	bool recordCameraPose(const Mat44& mat,double timestamp);
protected:
	std::ofstream _record_file;
};

#endif /* TRAJECTORYRECORDER_H_ */
