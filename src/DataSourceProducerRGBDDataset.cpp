/*
 * DataSourceProducerRGBDDataset.cpp
 *
 *  Created on: May 27, 2016
 *      Author: hanyinbo
 */

#include "DataSourceProducerRGBDDataset.h"

DataSourceProducerRGBDDataset::DataSourceProducerRGBDDataset():_depth_factor(5),_rgb_filename("rgb.txt"),_depth_filename("depth.txt"){
	// TODO Auto-generated constructor stub

}

DataSourceProducerRGBDDataset::~DataSourceProducerRGBDDataset() {
	// TODO Auto-generated destructor stub
	release();
}
void DataSourceProducerRGBDDataset::release()
{
	if (_rgbfile.is_open()) _rgbfile.close();
	if (_dptfile.is_open()) _dptfile.close();
}
bool DataSourceProducerRGBDDataset::recordFrameToFile()
{//not implement yet
	return false;
}
bool DataSourceProducerRGBDDataset::initRecorder(string record_filename)
{//not implement yet
	return false;
}

bool DataSourceProducerRGBDDataset::initDataSource()
{
	{//init depth src file
		string frame_depth_path = _sourcefilename + _depth_filename;
		_dptfile.open(frame_depth_path.c_str());
		string dpt_line;
		if(!_dptfile.is_open())return false;
		for(int i =0;i<3;i++) getline(_dptfile,dpt_line);
	}
    if(_capture_color)
    {//init color src file
    	string frame_rgb_path = _sourcefilename + _rgb_filename;
    	_rgbfile.open(frame_rgb_path.c_str());
    	string rgb_line;
    	if(!_rgbfile.is_open())return false;
		for(int i =0;i<3;i++)getline(_rgbfile,rgb_line);
    }
    return true;
}
bool DataSourceProducerRGBDDataset::parseFrameLine(ifstream& file ,double& timeStamp,string& path)
{
	if(file.eof())return false;
	string line;
	getline(file,line);
	path=_sourcefilename;
    stringstream ss(line);
    ss>>timeStamp;
    string temp_path;
    ss>>temp_path;
    path+=temp_path;
    return true;

}
bool DataSourceProducerRGBDDataset::parseFrameLineColor(ifstream& file ,double target_timestamp,double& timeStamp,string& path)
{
	string traj_line;
	double cur_timestamp;
//	cout<<setprecision(15) << target_timestamp<<endl;
	double last_timestamp=0;
	string last_path;
	//must be _trajfile_src.tellg.timestamp < target_timestamp
	while(!file.eof())
	{
		std::streampos file_pos = file.tellg();
		getline(file,traj_line);
		stringstream ss(traj_line);
		ss>>cur_timestamp;
		ss>>path;
		if(cur_timestamp>=target_timestamp)
		{
			if(cur_timestamp-target_timestamp>target_timestamp-last_timestamp)
			{//use last line data
				timeStamp=last_timestamp;
				path=_sourcefilename+last_path;
				file.seekg(file_pos);//go back to last line for convenience of next frame
			}
			else
			{
				timeStamp=cur_timestamp;
				path=_sourcefilename+path;
			}
			return true;
		}
		last_timestamp=cur_timestamp;
		last_path=path;
	}
    return false;

}
bool DataSourceProducerRGBDDataset::readDataFromSource(DepthFrameData& depth_data,ColorFrameData& rgb_data)
{//precondition : depth data captured after rgb data
	double cur_timestamp;
	string cur_frame_filepath;
	{
		if(false==parseFrameLine(_dptfile,cur_timestamp,cur_frame_filepath))return false;
		cv::Mat cur_depth_mat= cv::imread(cur_frame_filepath,CV_LOAD_IMAGE_UNCHANGED);
		if(cur_depth_mat.empty())return false;
		cur_depth_mat/=_depth_factor;
		depth_data.setFrameData(cur_depth_mat,cur_timestamp);

	}
	double target_timestamp=cur_timestamp;
	if(_capture_color)
	{
		if(false==parseFrameLineColor(_rgbfile,target_timestamp,cur_timestamp,cur_frame_filepath))return false;
        cv::Mat cur_rgb_mat= cv::imread(cur_frame_filepath);
        if(cur_rgb_mat.empty())return false;
        rgb_data.setFrameData(cur_rgb_mat,cur_timestamp);
	}
    return true;
}
