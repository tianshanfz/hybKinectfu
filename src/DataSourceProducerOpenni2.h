/*
 * DataSourceProducerOpenni2.h
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#ifndef DATASOURCEPRODUCEROPENNI2_H_
#define DATASOURCEPRODUCEROPENNI2_H_


#include<OpenNI.h>
#include"DataSourceProducer.h"
class DataSourceProducerOpenni2:public DataSourceProducer {
public:
	DataSourceProducerOpenni2(){};
	virtual ~DataSourceProducerOpenni2();
protected:
	bool initDataSource();
	bool initRecorder(string record_filename);
	bool readDataFromSource(DepthFrameData& depth_data,ColorFrameData& rgb_data);
	bool recordFrameToFile();
private:
	bool readOpenniFrame(openni::VideoFrameRef& openni_frame_depth,openni::VideoFrameRef& openni_frame_rgb);
	bool setupOpenNI(int depth_cols,int depth_rows,int rgb_cols,int rgb_rows);
	bool setupOpenNIVideoStreams(int depth_cols,int depth_rows,int rgb_cols,int rgb_rows);
	void releaseOpenNI();
	openni::Device _device;
	openni::VideoStream _stream_depth;
	openni::VideoStream _stream_color;
	openni::Recorder _recorder;
};

#endif /* DATASOURCEPRODUCEROPENNI2_H_ */
