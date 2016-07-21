/*
 * TrajectoryRecorder.cpp
 *
 *  Created on: May 31, 2016
 *      Author: hanyinbo
 */

#include "TrajectoryRecorder.h"
#include "Eigen/Eigen"
TrajectoryRecorder::TrajectoryRecorder(const string& record_filename)
 {
	_record_file.open(record_filename.c_str());
	if(!_record_file.is_open())return ;
	_record_file<<"# trajectory"<<endl;
	_record_file<<"# file: "<<record_filename<<endl;
	_record_file<<"# timestamp tx ty tz qx qy qz qw"<<endl;
}

TrajectoryRecorder::~TrajectoryRecorder() 
{
	if(_record_file.is_open())_record_file.close();
}
/*void Mat44ToQuant(const Mat44& mat,Eigen::Quaternion<float>& q)
{
	float qw=1;
	float qx=0.25*(mat.m23-mat.m32);
	float qy=0.25*(mat.m12+mat.m21);
	float qz=0.25*(mat.m31+mat.m13);
	q=Eigen::Quaternion<float>(qw,qx,qy,qz);
}*/
bool TrajectoryRecorder::recordCameraPose(const Mat44& mat,double timestamp)
{
	const float3 t=mat.getTranslation();
	const Mat33 rotation=mat.getRotation();
	Eigen::Matrix3f r_tran(rotation.entries);
	Eigen::Matrix3f r=r_tran.transpose();

	Eigen::Quaternion<float> q(r);
	_record_file<<setprecision(14)<<timestamp<<" ";
	_record_file<<setprecision(6)<<t.x<<" "<<t.y<<" "<<t.z<<" ";
	_record_file<<setprecision(6)<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
	return true;
}
