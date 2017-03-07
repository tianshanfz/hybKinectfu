/*
 * HybKinectfu.cpp
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#include "HybKinectfu.h"
#include "AppParams.h"
#include "cuda/CudaDeviceDataMan.h"
#include "cuda/CudaWrappers.h"
#include "CameraPoseFinderICP.h"
#include "CameraPoseFinderSDF.h"
#include "CameraPoseFinderFromFile.h"
#include "DataViewer.h"
#include "DataIO.h"
#include "keyframeMan.h"


HybKinectfu::HybKinectfu():_inited(false),_camera_pose_recorder(nullptr),_camera_pose_finder(nullptr) {
}

HybKinectfu::~HybKinectfu() {
	SAFE_DELETE(_camera_pose_recorder);
	SAFE_DELETE(_camera_pose_finder);
}

bool HybKinectfu::init()
{
	if(_inited)return false;
	if(AppParams::instance()->_switch_params.useTrajFromFile)
	{
		_camera_pose_finder=new CameraPoseFinderFromFile();
	}
	else
	{
		if(AppParams::instance()->_switch_params.useSdfTracker)
		{
			_camera_pose_finder=new CameraPoseFinderSDF();
		}
		else
		{
			_camera_pose_finder=new CameraPoseFinderICP();
		}

	}
	if(AppParams::instance()->_switch_params.recordTrajectory)
	{
		_camera_pose_recorder=new TrajectoryRecorder(AppParams::instance()->_io_params.trajWriteFilename);
	}
	Mat44 camera_pose0=Mat44::getIdentity();
	camera_pose0.setTranslation(make_float3(AppParams::instance()->_volume_params.fVolumeMeterSize/2.0,
											AppParams::instance()->_volume_params.fVolumeMeterSize/2.0,
											-AppParams::instance()->_depth_prepocess_params.fMinTrunc));


	if(false==_camera_pose_finder->init(camera_pose0)) return false;

	_inited=true;
	return _inited;
}

void HybKinectfu::copyFrameToGPU(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)
{
	cv::Mat mat_depth=depth_frame.mat();
	int rows=mat_depth.rows,cols=mat_depth.cols;
	DepthfMap2D depth_data_cpu(CPU,cols,rows);
	depth_data_cpu.clearData();
	for(int row=0;row<rows;row++)
	{
		for(int col=0;col<cols;col++)
		{
			float v=mat_depth.at<unsigned short>(row,col)*0.001;//millimeter to meters
			depth_data_cpu.at(col,row)=v;
		}
	}

	CudaDeviceDataMan::instance()->raw_depth.copyDataFrom(depth_data_cpu);

	if(AppParams::instance()->_switch_params.useRGBData)
	{
		cv::Mat mat_rgb=color_frame.mat();
		rows=color_frame.mat().rows,cols=color_frame.mat().cols;
		Color3uMap2D rgb_data_cpu(CPU,cols,rows);
		rgb_data_cpu.clearData();
		for(int row=0;row<rows;row++)
		{
			for(int col=0;col<cols;col++)
			{
				cv::Vec3b v=mat_rgb.at<cv::Vec3b>(row,col);
				rgb_data_cpu.at(col,row)=make_uchar3(v[0],v[1],v[2]);
			}
		}
		CudaDeviceDataMan::instance()->raw_rgb.copyDataFrom(rgb_data_cpu);
	}
}

bool HybKinectfu::processNewFrame(const DepthFrameData& depth_frame,const ColorFrameData& rgb_frame)
{
	if(!_inited)return false;
	double t0=clock();
	copyFrameToGPU(depth_frame,rgb_frame);
	cout<<"copy done"<<endl;
	DataViewer::viewDepths(CudaDeviceDataMan::instance()->raw_depth,"origin depth maps");
	DataViewer::viewColors(CudaDeviceDataMan::instance()->raw_rgb,"origin color maps");
	cudaTruncDepth(AppParams::instance()->_depth_prepocess_params.fMinTrunc,AppParams::instance()->_depth_prepocess_params.fMaxTrunc);
	cudaBiliearFilterDepth(AppParams::instance()->_depth_prepocess_params.fSigmaPixel,AppParams::instance()->_depth_prepocess_params.fSigmaDepth);
	DataViewer::viewDepths(CudaDeviceDataMan::instance()->filtered_depth,"filtered depth maps");
	cudaCalculateNewVertices(AppParams::instance()->_depth_camera_params);
	cudaCalculateNewNormals();
	cout<<"data preprocess cost"<<(clock()-t0)/1000.0<<"ms"<<endl;

	bool camera_tracking_success=false;

	t0=clock();
	camera_tracking_success=_camera_pose_finder->findCameraPose(depth_frame,rgb_frame);
	cout<<"camera tracking cost"<<(clock()-t0)/1000.0<<"ms"<<endl;//_cur_transform.print();
	Mat44 cur_camera_pose=_camera_pose_finder->getCameraPose();
	cout<<cur_camera_pose;



	if(camera_tracking_success)
	{
		if(depth_frame.frameId()%100==0)
		{
			KeyframeMan::instance()->addNewKeyFrame(depth_frame,rgb_frame,cur_camera_pose);
		}
		if(AppParams::instance()->_switch_params.recordTrajectory)
		{
			_camera_pose_recorder->recordCameraPose(cur_camera_pose,depth_frame.timeStamp());
		}
		t0=clock();
		cudaIntegrateVolume(AppParams::instance()->_switch_params.useRGBData,
							AppParams::instance()->_switch_params.colorAngleWeight,
							cur_camera_pose,
							AppParams::instance()->_integrate_params,
							AppParams::instance()->_depth_camera_params,
							AppParams::instance()->_rgb_camera_params);
		cout<<"integration cost"<<(clock()-t0)/1000.0<<"ms"<<endl;
	}
	else
	{
		cout<<"camera lost"<<endl;
		DataViewer::viewNormal(CudaDeviceDataMan::instance()->new_normals_pyramid[0],"new normal maps");
			cv::waitKey();
	}
	t0=clock();
	cudaRaycastingVolume(AppParams::instance()->_switch_params.useRGBData,
						cur_camera_pose,
						AppParams::instance()->_raycast_params,
						AppParams::instance()->_depth_camera_params,
						AppParams::instance()->_depth_prepocess_params.fMinTrunc,
						AppParams::instance()->_depth_prepocess_params.fMaxTrunc);
	cout<<"raycasting cost"<<(clock()-t0)/1000.0<<"ms"<<endl;
	DataViewer::viewNormal(CudaDeviceDataMan::instance()->new_normals_pyramid[0],"new normal maps");
	DataViewer::viewNormal(CudaDeviceDataMan::instance()->model_normals_pyramid[0],"model normal maps");
	DataViewer::viewColors(CudaDeviceDataMan::instance()->raycast_rgb,"raycast maps");
	return true;
}
