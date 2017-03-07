/*
 * CameraPoseFinderICP.cpp
 *
 *  Created on: May 29, 2016
 *      Author: hanyinbo
 */

#include "CameraPoseFinderICP.h"
#include "cuda/CudaWrappers.h"
#include "cuda/CudaDeviceDataMan.h"

bool CameraPoseFinderICP::initPoseFinder()
{
	unsigned levels=AppParams::instance()->_icp_params.nPyramidLevels;
	_iter_nums.resize(levels);
	if(levels==1)
	{
		_iter_nums[0]=3;
	}
	else if(levels==2)
	{
		_iter_nums[0]=10;
		_iter_nums[1]=5;
	}
	else if(levels==3)
	{
		_iter_nums[0]=10;
		_iter_nums[1]=5;
		_iter_nums[2]=4;
	}
	else
	{
		return false;
	}
	_camera_params_pyramid.push_back(AppParams::instance()->_depth_camera_params);
	for(int l=1;l<levels;l++)
	{
		CameraParams cur_params;
		cur_params.cols=_camera_params_pyramid[l-1].cols/2;
		cur_params.rows=_camera_params_pyramid[l-1].rows/2;
		cur_params.cx=_camera_params_pyramid[l-1].cx/2;
		cur_params.cy=_camera_params_pyramid[l-1].cy/2;
		cur_params.fx=_camera_params_pyramid[l-1].fx/2;
		cur_params.fy=_camera_params_pyramid[l-1].fy/2;
		_camera_params_pyramid.push_back(cur_params);
	}
	return true;

}
bool CameraPoseFinderICP::estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)
{
	if(depth_frame.frameId()==0)
	{
		return true;
	}
	//prepare datas
	cudaDownSampleNewVertices();
	cudaDownSampleNewNormals();
	cudaDownSampleModelVertices();
	cudaDownSampleModelNormals();

	Mat44 cur_transform=_pose;
	Mat44 last_transform_inv=_pose.getInverse();
	Eigen::Matrix<float, 6, 1, 0, 6, 1> delta_dof;
	for(int l=_iter_nums.size()-1;l>=0;l--)
	{
		int it_nums=_iter_nums[l];
		while(it_nums--)
		{
		//	findCorresSet(l,cur_transform,last_transform_inv);
			if(false==minimizePointToPlaneErrFunc(l,delta_dof,cur_transform,last_transform_inv)) return false;
			Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
			if (false==vector6ToTransformMatrix(delta_dof,  t) )
			{
				cout<<"camera shaking detected"<<endl;
				return false;
			}
			//eigen matrix to mat44
			Eigen::Matrix4f tt = t.transpose();
			Mat44 mat_t(tt.data());
			cur_transform = mat_t*cur_transform;
		}
	}
	_pose=cur_transform;
	return true;
}
/*void CameraPoseFinderICP::findCorresSet(unsigned level,const Mat44& cur_transform,const Mat44& last_transform_inv)
{
	 cudaProjectionMapFindCorrs(level,cur_transform,last_transform_inv,
			 _camera_params_pyramid[level],
			AppParams::instance()->_icp_params.fDistThres,
			AppParams::instance()->_icp_params.fNormSinThres);

}*/
bool CameraPoseFinderICP::vector6ToTransformMatrix(const Eigen::Matrix<float, 6, 1, 0, 6, 1>& x,  Eigen::Matrix4f& output)
{
	Eigen::Matrix3f R = Eigen::AngleAxisf(x[0], Eigen::Vector3f::UnitX()).toRotationMatrix()
		*Eigen::AngleAxisf(x[1], Eigen::Vector3f::UnitY()).toRotationMatrix()
		*Eigen::AngleAxisf(x[2], Eigen::Vector3f::UnitZ()).toRotationMatrix();
	Eigen::Vector3f t = x.segment(3, 3);
	Eigen::AngleAxisf aa(R);
	float angle = aa.angle();
	float d = t.norm();
	if (angle > AppParams::instance()->_icp_params.fAngleShake|| d> AppParams::instance()->_icp_params.fDistShake)
	{
		return false;
	}
	output.block(0, 0, 3, 3) = R;
	output.block(0, 3, 3, 1) = t;
	return true;
}

bool CameraPoseFinderICP::minimizePointToPlaneErrFunc(unsigned level,Eigen::Matrix<float, 6, 1> &six_dof,const Mat44& cur_transform,const Mat44& last_transform_inv)
{
	cudaCalPointToPlaneErrSolverParams( level,cur_transform,last_transform_inv, _camera_params_pyramid[level],AppParams::instance()->_icp_params.fDistThres,
			AppParams::instance()->_icp_params.fNormSinThres);
	CudaMap1D<float> buf=(CudaDeviceDataMan::instance()->rigid_align_buf_reduced).clone(CPU);
	int shift = 0;
	Eigen::Matrix<float, 6, 6, Eigen::RowMajor> ATA;
	Eigen::Matrix<float, 6, 1> ATb;
	for (int i = 0; i < 6; ++i)  //rows
	{
		for (int j = i; j < 7; ++j)    // cols + b
		{
			float value = buf.at(shift++);
			if (j == 6)
			{
				ATb(i) = value;
			}
			else
			{
				ATA(i, j) = value;
				ATA(j, i) = value;
			}
		}
	}
	//cout<<ATb<<endl;
	if (ATA.determinant()<1E-10)
	{
		return false;
	}

	six_dof = ATA.llt().solve(ATb).cast<float>();
	return true;
}
