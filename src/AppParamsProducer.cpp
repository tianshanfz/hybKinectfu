/*
 * AppParamsProducer.cpp
 *
 *  Created on: May 25, 2016
 *      Author: hanyinbo
 */

#include "AppParamsProducer.h"

AppParamsProducer::AppParamsProducer() {
	// TODO Auto-generated constructor stub

}

AppParamsProducer::~AppParamsProducer() {
	// TODO Auto-generated destructor stub
}
bool AppParamsProducer::readParams(AppParams* params,string config_filename)
{
	_ini_parser.clear();
	if(false==_ini_parser.readINI(config_filename))	return false;
	stringstream str_stream;
	//[switch]
	str_stream<<_ini_parser.getValue("switch", "use_color");
	str_stream>>params->_switch_params.useRGBData;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("switch", "record_rgbd") ;
	str_stream>>params->_switch_params.recordRGBD;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("switch", "record_trajectory") ;
	str_stream>>params->_switch_params.recordTrajectory;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("switch", "color_angle_weight") ;
	str_stream>>params->_switch_params.colorAngleWeight;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("switch", "read_rgbd_from_rgbddataset") ;
	str_stream>>params->_switch_params.useDatasetRGBD;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("switch", "read_trajactory_from_file") ;
	str_stream>>params->_switch_params.useTrajFromFile;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("switch", "use_sdf_tracker") ;
	str_stream>>params->_switch_params.useSdfTracker;
	str_stream.clear();

	//[camera]
	str_stream<<_ini_parser.getValue("camera", "rgb_cols");
	str_stream>>params->_rgb_camera_params.cols;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "rgb_rows");
	str_stream>>params->_rgb_camera_params.rows;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "rgb_cx");
	str_stream>>params->_rgb_camera_params.cx;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "rgb_cy");
	str_stream>>params->_rgb_camera_params.cy;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "rgb_fx");
	str_stream>>params->_rgb_camera_params.fx;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "rgb_fy");
	str_stream>>params->_rgb_camera_params.fy;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "depth_cols");
	str_stream>>params->_depth_camera_params.cols;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "depth_rows");
	str_stream>>params->_depth_camera_params.rows;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "depth_cx");
	str_stream>>params->_depth_camera_params.cx;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "depth_cy");
	str_stream>>params->_depth_camera_params.cy;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "depth_fx");
	str_stream>>params->_depth_camera_params.fx;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("camera", "depth_fy");
	str_stream>>params->_depth_camera_params.fy;
	str_stream.clear();
	//kinectfusion
	str_stream<<_ini_parser.getValue("kinectfusion", "depth_trunc_max");
	str_stream>>params->_depth_prepocess_params.fMaxTrunc;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "depth_trunc_min");
	str_stream>>params->_depth_prepocess_params.fMinTrunc;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "filter_sigma_pixel");
	str_stream>>params->_depth_prepocess_params.fSigmaPixel;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "filter_sigma_depth");
	str_stream>>params->_depth_prepocess_params.fSigmaDepth;
	str_stream.clear();

	str_stream<<_ini_parser.getValue("kinectfusion", "volume_resolution");
	str_stream>>params->_volume_params.nResolution;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "volume_size_meter");
	str_stream>>params->_volume_params.fVolumeMeterSize;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "volume_max_weight");
	str_stream>>params->_volume_params.fWeightMax;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "integrate_sdf_trunc");
	str_stream>>params->_integrate_params.fSdfTruncation;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("kinectfusion", "integrate_depth_trunc");
	str_stream>>params->_integrate_params.fMaxIntegrateDist;
	str_stream.clear();

	str_stream<<_ini_parser.getValue("kinectfusion", "raycast_increment_factor");
	float factor;
	str_stream>>factor;
	str_stream.clear();
	params->_raycast_params.fRayIncrement=factor*params->_integrate_params.fSdfTruncation;
	str_stream.clear();


	//[icp]
	str_stream<<_ini_parser.getValue("icp", "icp_pyramid_level");
	str_stream>>params->_icp_params.nPyramidLevels;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("icp", "icp_thre_dist");
	str_stream>>params->_icp_params.fDistThres;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("icp", "icp_thre_sin_angle");
	str_stream>>params->_icp_params.fNormSinThres;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("icp", "camera_shake_dist");
	str_stream>>params->_icp_params.fDistShake;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("icp", "camera_shake_angle");
	str_stream>>params->_icp_params.fAngleShake;
	str_stream.clear();

	//[sdfTracker]
	str_stream<<_ini_parser.getValue("sdfTracker", "max_iter_nums");
	str_stream>>params->_sdf_tracker_params.maxIterNums;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("sdfTracker", "camera_shake_dist");
	str_stream>>params->_sdf_tracker_params.fDistShake;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("sdfTracker", "camera_shake_angle");
	str_stream>>params->_sdf_tracker_params.fAngleShake;
	str_stream.clear();

	//[MESH]
	str_stream<<_ini_parser.getValue("mesh", "max_triangle_num");
	str_stream>>params->_marchingcube_params.uMaxTriangles;
	str_stream.clear();

	//[IO]
	str_stream<<_ini_parser.getValue("IO", "rgbd_record_filename");
	str_stream>>params->_io_params.rgbdWriteFilename;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("IO", "rgbd_source_filename");
	str_stream>>params->_io_params.rgbdReadFilename;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("IO", "trajectory_record_filename");
	str_stream>>params->_io_params.trajWriteFilename;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("IO", "trajectory_source_filename");
	str_stream>>params->_io_params.trajReadFilename;
	str_stream.clear();
	str_stream<<_ini_parser.getValue("IO", "mesh_filename");
	str_stream>>params->_io_params.meshFilename;
	str_stream.clear();
	return true;
}


