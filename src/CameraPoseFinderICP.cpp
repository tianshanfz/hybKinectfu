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
		_iter_nums[0]=15;
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
			findCorresSet(l,cur_transform,last_transform_inv);
			if(false==minimizePointToPlaneErrFunc(l,delta_dof)) return false;
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
void CameraPoseFinderICP::findCorresSet(unsigned level,const Mat44& cur_transform,const Mat44& last_transform_inv)
{
	 cudaProjectionMapFindCorrs(level,cur_transform,last_transform_inv,
			 _camera_params_pyramid[level],
			AppParams::instance()->_icp_params.fDistThres,
			AppParams::instance()->_icp_params.fNormSinThres);

}
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
bool CameraPoseFinderICP::minimizePointToPlaneErrFuncHost(unsigned level,Eigen::Matrix<float, 6, 1> &six_dof)
{//for debugging
	Vector4fMap2D input_v,target_v;
	Point4fMap2D input_n;
	input_v.create_cpu(CudaDeviceDataMan::instance()->_model_vertices_pyramid[level]);
	input_n.create_cpu(CudaDeviceDataMan::instance()->_model_normals_pyramid[level]);
	target_v.create_cpu(CudaDeviceDataMan::instance()->_corrs_vertices_pyramid[level]);
	int cols=input_v.cols(),rows=input_v.rows();
	int shift = 0;
	Eigen::Matrix<float, 6, 6, Eigen::RowMajor> ATA;
	ATA<<0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0,
		0,0,0,0,0,0;
	Eigen::Matrix<float, 6, 1> ATb;
	ATb<<0,0,0,0,0,0;
	for(int y=0;y<rows;y++)
	for(int x=0;x<cols;x++)
	{
		float4 q4=target_v.get_data(x, y);
		float3 q = make_float3(q4.x,q4.y,q4.z);
		float4 p4=input_v.get_data(x, y);
		float3 p = make_float3(p4.x,p4.y,p4.z);
		float4 n4=input_n.get_data(x, y);
		float3 n = make_float3(n4.x,n4.y,n4.z);
		if(isZero(n)||q.z==0)	continue;
		float row[7] = { 0, 0, 0, 0, 0, 0 ,0};
		row[0] = q.y*n.z - q.z*n.y;
		row[1] = q.z*n.x - q.x*n.z;
		row[2] = q.x*n.y - q.y*n.x;
		row[3] = n.x;
		row[4] = n.y;
		row[5] = n.z;
		row[6] = dot(n, p - q);
		for (int i = 0; i < 6; ++i)  //rows
		{
			for (int j = i; j < 7; ++j)    // cols + b
			{
				float value =row[i]*row[j];
				if (j == 6)ATb(i) += value;
				else
				{
					ATA(i, j) += value;
					ATA(j, i) += value;
				}
			}
		}
	}

	input_v.destroy();
	input_n.destroy();
	target_v.destroy();
	if (ATA.determinant()<1E-10) return false;
	six_dof = ATA.llt().solve(ATb).cast<float>();
	return true;
}
bool CameraPoseFinderICP::minimizePointToPlaneErrFunc(unsigned level,Eigen::Matrix<float, 6, 1> &six_dof)
{
	cudaCalPointToPlaneErrSolverParams(  level);
	DataMap1D<float> buf;
	buf.create_cpu(CudaDeviceDataMan::instance()->_rigid_align_buf_reduced);
	int shift = 0;
	Eigen::Matrix<float, 6, 6, Eigen::RowMajor> ATA;
	Eigen::Matrix<float, 6, 1> ATb;
	for (int i = 0; i < 6; ++i)  //rows
	{
		for (int j = i; j < 7; ++j)    // cols + b
		{
			float value = buf.get_data(shift++);
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
		buf.destroy();
		return false;
	}

	six_dof = ATA.llt().solve(ATb).cast<float>();
	buf.destroy();
	return true;
}
