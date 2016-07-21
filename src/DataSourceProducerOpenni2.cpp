/*
 * DataSourceProducerOpenni2.cpp
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#include "DataSourceProducerOpenni2.h"
#include<opencv2/opencv.hpp>




DataSourceProducerOpenni2::~DataSourceProducerOpenni2()
{
	releaseOpenNI();
}
void DataSourceProducerOpenni2::releaseOpenNI()
{
	cout<<"release openni device..."<<endl;
	_stream_depth.destroy();
	if(_capture_color)_stream_color.destroy();
	_recorder.stop();
	_recorder.destroy();
	_device.close();

	openni::OpenNI::shutdown();

}
bool DataSourceProducerOpenni2::initDataSource()
{
	return setupOpenNI(AppParams::instance()->_depth_camera_params.cols,
							AppParams::instance()->_depth_camera_params.rows,
							AppParams::instance()->_rgb_camera_params.cols,
							AppParams::instance()->_rgb_camera_params.rows);
}
bool DataSourceProducerOpenni2::readOpenniFrame(openni::VideoFrameRef& openni_frame_depth,openni::VideoFrameRef& openni_frame_rgb)
{

	
	if(openni::STATUS_ERROR== _stream_depth.start())return false;
	if(openni::STATUS_ERROR==_stream_depth.readFrame(&openni_frame_depth))return false;
	_stream_depth.stop();
	
	if(_capture_color)
	{
		if(openni::STATUS_ERROR==_stream_color.start())return false;
		if(openni::STATUS_ERROR==_stream_color.readFrame(&openni_frame_rgb))return false;
		_stream_color.stop();
	}
	return true;
}
bool DataSourceProducerOpenni2::initRecorder(string record_filename)
{
	if(openni::STATUS_OK!=_recorder.create(record_filename.c_str()))return false;
	if(openni::STATUS_OK!=_recorder.attach(_stream_depth))return false;
	if(_capture_color)
	{
		if(openni::STATUS_OK!=_recorder.attach(_stream_color))return false;
	}
	if(openni::STATUS_OK!=_recorder.start())return false;
	return true;
}
bool DataSourceProducerOpenni2::recordFrameToFile()
{//openni recorder is recording automatically
	return _recorder.isValid();
}

bool DataSourceProducerOpenni2::readDataFromSource(DepthFrameData& depth_data,ColorFrameData& rgb_data)
{
	openni::VideoFrameRef  frame_depth,frame_color;
	if(false==readOpenniFrame(frame_depth,frame_color)) return false;
	depth_data.setFrameData((void *)frame_depth.getData(), frame_depth.getWidth(), frame_depth.getHeight(),frame_depth.getTimestamp());
	if(_capture_color)
	{
		rgb_data.setFrameData((void*)frame_color.getData(), frame_color.getWidth(), frame_color.getHeight(),frame_color.getTimestamp());
	}

	return true;
}
bool DataSourceProducerOpenni2::setupOpenNIVideoStreams(int depth_cols,int depth_rows,int rgb_cols,int rgb_rows)
{
	//set up rgb stream
	if(_capture_color)
	{
		if(openni::STATUS_ERROR ==_stream_color.create(_device, openni::SENSOR_COLOR))return false;
		if(_sourcefilename=="null")
		{
			openni::VideoMode rgb_video_mode;
			rgb_video_mode.setResolution(rgb_cols, rgb_rows);
			rgb_video_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
		//	rgb_video_mode.setFps(30);
			if(openni::STATUS_ERROR==_stream_color.setVideoMode(rgb_video_mode))return false;
		}
	}
	//set up depth stream
	if(openni::STATUS_OK != _stream_depth.create(_device, openni::SENSOR_DEPTH))return false;
	if(_sourcefilename=="null")
	{
		openni::VideoMode depth_video_mode;
		depth_video_mode.setResolution(depth_cols, depth_rows);
		depth_video_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		depth_video_mode.setFps(30);
		if(openni::STATUS_ERROR==_stream_depth.setVideoMode(depth_video_mode))return false;
	}
	return true;
}
bool DataSourceProducerOpenni2::setupOpenNI(int depth_cols,int depth_rows,int rgb_cols,int rgb_rows)
{
	openni::Status status;
	if(openni::STATUS_OK!= openni::OpenNI::initialize())
	{
		cout<<openni::OpenNI::getExtendedError();
		return false;
	}
	//set up device
	const char* device_name=(_sourcefilename=="null"?openni::ANY_DEVICE:_sourcefilename.c_str());
	if(openni::STATUS_OK != _device.open(device_name))
	{
		cout<<openni::OpenNI::getExtendedError();
		return false;
	}
	if(false==setupOpenNIVideoStreams(depth_cols,depth_rows,rgb_cols,rgb_rows)) return false;

	if(_capture_color)
	{
		//if(openni::STATUS_ERROR ==_device.setDepthColorSyncEnabled(true))return false;
		if( _device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
		{
			if(openni::STATUS_ERROR==_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))return false;
		}
	}
	return true;
}
