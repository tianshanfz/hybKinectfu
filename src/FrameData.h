/*
 * FrameData.h
 *
 *  Created on: May 26, 2016
 *      Author: hanyinbo
 */

#ifndef FRAMEDATA_H_
#define FRAMEDATA_H_


#include<opencv/cv.h>
#include<opencv/highgui.h>
#include<string>
using namespace std;
class FrameData
{
public:
	FrameData(){}
	FrameData(unsigned frameId):_frame_id(frameId){}
	virtual void setFrameData(void* data,int cols,int rows,double time_stamp)=0;
	virtual void setFrameData( cv::Mat& mat,double time_stamp)=0;
	virtual bool saveToFile(const string& filename)const
	{
       if(_data_mat.empty())return false;
		cv::imwrite(filename,_data_mat);
		return true;
	}
	virtual bool readFromFile(const string& filename)
	{
		_data_mat=cv::imread(filename);
		return !_data_mat.empty();
	}
	virtual void showFrame(const char* window_name)const=0;

	virtual void copyTo(FrameData& frame_data)const
	{
		frame_data._time_stamp=_time_stamp;
		frame_data._is_setted=_is_setted;
		frame_data._frame_id=_frame_id;
		copyMatTo(frame_data._data_mat);
	}

	double timeStamp()const {return _time_stamp;}
	virtual ~FrameData(){}
	const unsigned frameId()const{ return _frame_id;}
	const cv::Mat& mat()const{return _data_mat;}
	 cv::Mat& mat(){return _data_mat;}
protected:
	virtual void copyMatTo(cv::Mat &mat)const=0;
	cv::Mat		_data_mat;
	double 		_time_stamp;
	bool		_is_setted;
	unsigned 	_frame_id;
};
class DepthFrameData :public FrameData
{//depth in millimeter(unsigned short), 0 indicates invalid value
public:
	DepthFrameData():FrameData(){}
	DepthFrameData(unsigned frameId):FrameData(frameId){}
	virtual ~DepthFrameData(){}
	void setFrameData( cv::Mat& mat,double time_stamp)
	{
		_data_mat=mat;
		_time_stamp=time_stamp;
		_is_setted=true;
	}
	void setFrameData(void* data,int cols,int rows,double time_stamp)
	{
		_data_mat=cv::Mat(rows,cols,CV_16UC1,data);
		_time_stamp=time_stamp;
		_is_setted=true;
	}

	void showFrame(const char* window_name)const
	{
        if(_data_mat.empty())return ;
		cv::Mat image_show;
		double min_val,max_val;
		cv::minMaxIdx(_data_mat,&min_val,&max_val);
		_data_mat.convertTo( image_show, CV_8U, 255.0 / max_val );
		cv::imshow(window_name, image_show);
	}
protected:
	void copyMatTo(cv::Mat &mat)const
	{
		_data_mat.copyTo(mat);
	}
};
class ColorFrameData:public FrameData
{
public:
	ColorFrameData():FrameData(){}
	ColorFrameData(unsigned frameId):FrameData(frameId){}
	virtual ~ColorFrameData(){}
	void setFrameData( cv::Mat& mat,double time_stamp)
	{
		_data_mat=mat;
		_time_stamp=time_stamp;
	}
	void setFrameData(void* data,int cols,int rows,double time_stamp)
	{
		_data_mat=cv::Mat(rows,cols,CV_8UC3,data);
	//	cv::cvtColor(_data_mat, _data_mat, CV_RGB2BGR);//data from openni2
	}

	void showFrame(const char* window_name)const
	{
		if(_data_mat.empty())return ;
	//	cv::Mat image_show;
	//	cv::cvtColor(_data_mat, image_show, CV_RGB2BGR);
		cv::imshow(window_name, _data_mat);
	}
protected:
	void copyMatTo(cv::Mat &mat)const
	{
		_data_mat.copyTo(mat);
	}
};


/*
template<typename T>
struct FrameData
{
public:
	FrameData(unsigned cols,unsigned rows,unsigned channels,unsigned frameId,const T& max_value):_cols(cols),_rows(rows),_channels(channels),_max_value(max_value),_frame_id(frameId)
	{
		_ptr_data=new T[cols*rows*channels];
	}
	virtual ~FrameData(){
		SAFE_DELETE_ARR(_ptr_data);
	}
	unsigned 	frameId()const{return _frame_id;}
	unsigned 	cols()const{return _cols;}
	unsigned 	rows()const{return _rows;}
	unsigned	channels()const{return _channels;}
	const T*	ptr()const{return _ptr_data;}
	T*			ptr(){return _ptr_data;}
	void showFrame(const char* window_name)
	{
		cv::Mat image_show;
		if(3==_channels)
		{
			cv::Mat  image_rgb = cv::Mat(_rows,_cols, CV_8UC3, (void*)_ptr_data);
			cv::cvtColor(image_rgb, image_show, CV_RGB2BGR);
		}
		else if(1==_channels)
		{
			cv::Mat image_depth= cv::Mat(_rows,_cols, CV_32F, (void*)_ptr_data);
			image_depth.convertTo( image_show, CV_8U, 255.0 / _max_value );
		}
		cv::imshow(window_name, image_show);
		cv::waitKey(1);
	}
protected:
	const T		_max_value;
	T* 			_ptr_data;
	unsigned 	_cols;
	unsigned 	_channels;
	unsigned 	_rows;
	unsigned 	_frame_id;
};*/

#endif // FRAMEDATA_H_INCLUDED
