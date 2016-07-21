/*
 * DataSourceProducer.h
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#ifndef DATASOURCEPRODUCER_H_
#define DATASOURCEPRODUCER_H_
#include"utils/cpu_include.h"
#include"AppParams.h"
#include"FrameData.h"
class DataSourceProducer {
public:
	DataSourceProducer():_inited(false),_recording(false),_sourcefilename(""),_capture_color(false){}
	bool init()
	{
		if(_inited)return false;
		_sourcefilename=AppParams::instance()->_io_params.rgbdReadFilename;
		_capture_color=AppParams::instance()->_switch_params.useRGBData;
		_recording=AppParams::instance()->_switch_params.recordRGBD;
		if(false==initDataSource())return false;
		if(_recording)
		{
			if(false==initRecorder(AppParams::instance()->_io_params.rgbdWriteFilename))return false;
		}
		_inited=true;
		return _inited;
	}

	bool readNewFrame(DepthFrameData& depth_data,ColorFrameData& rgb_data)
	{
		if(!_inited)return false;
		if(false==readDataFromSource(depth_data,rgb_data))return false;
		if(_recording)return recordFrameToFile();
	}

	virtual ~DataSourceProducer(){}
protected:
	virtual bool initDataSource()=0;
	virtual bool readDataFromSource(DepthFrameData& depth_data,ColorFrameData& rgb_data)=0;
	virtual bool recordFrameToFile()=0;
	virtual bool initRecorder(string record_filename)=0;

	string _sourcefilename;
	bool _capture_color;
private:
	bool _inited;
	bool _recording;
};

#endif /* DATASOURCEPRODUCER_H_ */
