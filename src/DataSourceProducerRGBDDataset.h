/*
 * DataSourceProducerRGBDDataset.h
 *
 *  Created on: May 27, 2016
 *      Author: hanyinbo
 */

#ifndef DATASOURCEPRODUCERRGBDDATASET_H_
#define DATASOURCEPRODUCERRGBDDATASET_H_
#include"DataSourceProducer.h"
#include <fstream>

class DataSourceProducerRGBDDataset :public DataSourceProducer
{//read rgb and depth from rgbd slam dataset 
public:
	DataSourceProducerRGBDDataset();
	virtual ~DataSourceProducerRGBDDataset();
protected:
	bool initDataSource();
	bool readDataFromSource(DepthFrameData& depth_data,ColorFrameData& rgb_data);
	bool recordFrameToFile();
	bool initRecorder(string record_filename);
private:
	bool parseFrameLine(ifstream& file ,double& timeStamp,string& path);
	bool parseFrameLineColor(ifstream& file ,double target_timestamp,double& timeStamp,string& path);

	void release();
	const char *_rgb_filename;
	const float _depth_factor;
	const char *_depth_filename;
	std::ifstream _rgbfile;
	std::ifstream _dptfile;
};

#endif /* DATASOURCEPRODUCERRGBDDATASET_H_ */
