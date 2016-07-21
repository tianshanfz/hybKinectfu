/*
 * MainController.h
 *
 *  Created on: May 25, 2016
 *      Author: hanyinbo
 */

#ifndef MAINCONTROLLER_H_
#define MAINCONTROLLER_H_
#include "utils/cpu_declar.h"
#include "DataSourceProducer.h"
#include "AppParamsProducer.h"
#include "cuda/CudaDeviceDataMan.h"
#include "HybKinectfu.h"
#include "MeshGenerator.h"
class MainController {


public:
	static MainController* instance()
	{
		static MainController value;
		return &value;
	}
	bool init(string configfilename);
	void mainLoop();
protected:
	MainController();
	virtual ~MainController();
	AppParamsProducer* _appparams_producer;
	HybKinectfu* _kinect_fusioner;
	DataSourceProducer* _data_source;
	MeshGenerator *_mesh_generator;
	bool _inited;
};

#endif /* MAINCONTROLLER_H_ */
