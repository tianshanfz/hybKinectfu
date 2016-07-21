/*
 * MainController.cpp
 *
 *  Created on: May 25, 2016
 *      Author: hanyinbo
 */

#include "MainController.h"
#include "cuda/CudaDeviceDataMan.h"
#include "DataSourceProducerOpenni2.h"
#include "DataSourceProducerRGBDDataset.h"
#include "MeshGeneratorMarchingcube.h"
//#include "MeshGeneratorRaycasting.h"
//#include "keyframeMan.h"
MainController::MainController():_inited(false) {
	//_data_source=new DataSourceProducerRGBDDataset();
	_kinect_fusioner=nullptr;
	_mesh_generator=nullptr;
	_data_source=nullptr;
	_appparams_producer=nullptr;
}

MainController::~MainController() {
	SAFE_DELETE(_mesh_generator);
	SAFE_DELETE(_data_source);
	SAFE_DELETE(_kinect_fusioner);
	SAFE_DELETE(_appparams_producer);
}
void MainController::mainLoop()
{
	if(!_inited)return;
	for(int i=0;i<3000;i++)
	{
		cout<<"frame "<<i<<endl;
		DepthFrameData depth_frame(i);
		ColorFrameData rgb_frame(i);
		if(false==_data_source->readNewFrame(depth_frame,rgb_frame))
		{
			cout<<"read new frame failed at frame "<<i<<endl;
			break;
		}
		depth_frame.showFrame("depth frame");
		rgb_frame.showFrame("rgb frame");
		if(false==_kinect_fusioner->processNewFrame(depth_frame, rgb_frame))
		{
			cout<<"process kinectfusion failed at frame "<<i<<endl;
			break;
		}

		char c=cv::waitKey();
		if('q'==c)break;
		else if('p'==c)
		{
			c=cv::waitKey();
		}
		else if('s'==c)
		{
			double t0=clock();
			_mesh_generator->generateMesh();
			cout<<"generate mesh cost"<<(clock()-t0)/1000.0<<"ms"<<endl;//_cur_transform.print();
			if(false==_mesh_generator->saveMesh(AppParams::instance()->_io_params.meshFilename))
			{
				cout<<"save mesh failed"<<endl;
			}
		//	KeyframeMan::instance()->writeToFile("output/keyframe/idx.txt");

		}

	}

}

bool MainController::init(string configfilename)
{
	if(_inited)return false;

	_appparams_producer=new AppParamsProducer();
	if(false == _appparams_producer->readParams(AppParams::instance(),configfilename))
	{
		cout<<"read config.ini failed"<<endl;
		return false;
	}
	else AppParams::instance()->print();
	if(false==CudaDeviceDataMan::instance()->init())
	{
		cout<<"cuda data init failed"<<endl;
		return false;
	}
	if(AppParams::instance()->_switch_params.useDatasetRGBD)
	{
		_data_source=new DataSourceProducerRGBDDataset();
	}
	else
	{
		_data_source=new DataSourceProducerOpenni2();
	}
	if(false==_data_source->init())
	{
		cout<<"init data source failed"<<endl;
		return false;
	}
	_kinect_fusioner=new HybKinectfu();
	if(false==_kinect_fusioner->init())
	{
		cout<<"init kinect fusioner failed"<<endl;
		return false;
	}
	_mesh_generator=new MeshGeneratorMarchingcube();
	_inited=true;
	return true;
}
