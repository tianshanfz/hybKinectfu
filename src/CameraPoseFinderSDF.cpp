/*
 * CameraPoseFinderSDF.cpp
 *
 *  Created on: Jun 2, 2016
 *      Author: hanyinbo
 */

#include "CameraPoseFinderSDF.h"
#include "cuda/CudaWrappers.h"
#include "cuda/CudaDeviceDataMan.h"
#include "utils/eigen_utils.h"
CameraPoseFinderSDF::CameraPoseFinderSDF() {
	// TODO Auto-generated constructor stub

}

CameraPoseFinderSDF::~CameraPoseFinderSDF() {
	// TODO Auto-generated destructor stub
}

bool CameraPoseFinderSDF::initPoseFinder()
{
	return true;
}
bool CameraPoseFinderSDF::vector6ToTransformMatrix(Eigen::Matrix<float, 6, 1, 0, 6, 1>& x,  Eigen::Matrix4f& output)
{
	Eigen::Matrix3f R = Eigen::AngleAxisf(x[0], Eigen::Vector3f::UnitX()).toRotationMatrix()
		*Eigen::AngleAxisf(x[1], Eigen::Vector3f::UnitY()).toRotationMatrix()
		*Eigen::AngleAxisf(x[2], Eigen::Vector3f::UnitZ()).toRotationMatrix();

	Eigen::Vector3f t = x.segment(3, 3);
	Eigen::AngleAxisf aa(R);
	float angle = aa.angle();
	float d = t.norm();
	if (angle > AppParams::instance()->_sdf_tracker_params.fAngleShake|| d> AppParams::instance()->_sdf_tracker_params.fDistShake)
	{
		return false;
	}

	output.block(0, 0, 3, 3) = R;
	output.block(0, 3, 3, 1) = t ;
	return true;
}
bool CameraPoseFinderSDF::estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)
{
	if(depth_frame.frameId()==0)
	{
		return true;
	}
	Eigen::Matrix<float, 6, 6, Eigen::RowMajor> A;
	Eigen::Matrix<float, 6, 1>  b;
	Eigen::Matrix<float, 6, 1>  c;
	int iter = 0;
	float e = 0.001;

	Mat44 cur_transform = _pose;
	while (iter<AppParams::instance()->_sdf_tracker_params.maxIterNums)
	{
		int validCount=0;
		cudaCalSDFSolverParams( AppParams::instance()->_depth_camera_params, cur_transform);
		CudaMap1D<float> buf=(CudaDeviceDataMan::instance()->rigid_align_buf_reduced).clone(CPU);
		int shift = 0;
		for (int i = 0; i < 6; ++i)  //rows
		{
			for (int j = i; j < 7; ++j)  //cols && b
			{
				float value = buf.at(shift++);
				if (j == 6)
				{
					b(i) = value;
				}
				else
				{
					A(i, j) = value;
					A(j, i) = value;
				}
			}
		}
		Eigen::Matrix<float, 6, 1> x = A.llt().solve(b).cast<float>();
		Eigen::Matrix4f t = Eigen::Matrix4f::Identity();

		if (false==vector6ToTransformMatrix(x,  t) )
		{
			cout<<"camera shaking detected"<<endl;
			return false;
		}
		if(x.norm()<e)
		{
			break;
		}
		Eigen::Matrix<double, 6, 1> twist_diff = x.cast<double>();
		Eigen::Affine3d aff = eigen_utils::direct_exponential_map(twist_diff, 1.0);
		Eigen::Matrix4f curTrans_eigent(cur_transform.entries);
		Eigen::Matrix4f curTrans_eigen = curTrans_eigent.transpose();
	//	cout << curTrans_eigen << endl;
		Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
		temp.block(0, 0, 3, 3) = aff.rotation().transpose().cast<float>()*curTrans_eigen.block(0, 0, 3, 3);
		temp.block(0, 3, 3, 1) = curTrans_eigen.block(0, 3, 3, 1) - aff.rotation().transpose().cast<float>()*aff.translation().cast<float>();
		Eigen::Matrix4f temp_t = temp.transpose();
		cur_transform=Mat44(temp_t.data());
		iter++;
	}
	_pose=cur_transform;

	return true;
}
