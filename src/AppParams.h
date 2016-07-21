/*
 * AppParams.h
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#ifndef APPPARAMS_H_
#define APPPARAMS_H_

#include"utils/cpu_include.h"
struct IOParams
{
	string 		meshFilename;
	string		rgbdReadFilename;
	string 		rgbdWriteFilename;
	string		trajReadFilename;
	string		trajWriteFilename;
};
struct IcpParams
{
	unsigned 	nPyramidLevels;
	float 		fNormSinThres;
	float	 	fDistThres;
	float		fDistShake;
	float		fAngleShake;
//	IcpParams():fNormSinThres(0.2),fDistThres(0.1){}
//	IcpParams(float norm_thres,float dist_thres):fNormSinThres(norm_thres),fDistThres(dist_thres){}
};
struct SDFTrackerParams
{
	unsigned 	maxIterNums;
	float		fDistShake;
	float		fAngleShake;
};
struct CameraParams
{
	unsigned 	cols;//in pixels
	unsigned	rows;
	float		cx;	//in pixels
	float 		cy;
	float		fx;//in pixels
	float		fy;
	CameraParams operator* (int rate)const
	{
		CameraParams ret;
		ret.cols=cols*rate;
		ret.rows=rows*rate;
		ret.cx=cx*rate;
		ret.cy=cy*rate;
		ret.fx=fx*rate;
		ret.fy=fy*rate;
		return ret;
	}
};

struct RayCasterParams
{
	float 	fRayIncrement;
};
struct DepthPrepocessParams
{
	float 	fMaxTrunc; //in meters
	float 	fMinTrunc; //in meters
	float	fSigmaDepth;
	float   fSigmaPixel;
};
struct tsdfVolumeParams
{
	unsigned 	nResolution;
	float 		fVolumeMeterSize;
	float 		fWeightMax;
};

struct MarchingcubeParams
{
//	bool 		bColored;
	unsigned 	uMaxTriangles;
//	float  		fThreshMarchingCubes;
};
struct IntegrateParams
{
	float		fSdfTruncation;
	float 		fMaxIntegrateDist;
};
struct SwitchParams
{
	bool recordRGBD;
	bool recordTrajectory;
	bool useRGBData;
	bool colorAngleWeight;
	bool useDatasetRGBD;
	bool useTrajFromFile;
	bool useSdfTracker;
};
class AppParams {

public:
    static AppParams* instance()
    {
    	static AppParams value;
    	return &value;
    }
	void print()
	{
		cout<<"use rgb data="<<_switch_params.useRGBData<<endl;
		cout<<"use rgbd recording="<<_switch_params.recordRGBD<<endl;
		cout<<"use trajectory recording="<<_switch_params.recordTrajectory<<endl;
		cout<<"use angle weight for color="<<_switch_params.colorAngleWeight<<endl;
		cout<<"use rgbd data from dataset="<<_switch_params.useDatasetRGBD<<endl;
		cout<<"use trajectory from file="<<_switch_params.useTrajFromFile<<endl;

		cout<<"depth camera cols="<<_depth_camera_params.cols<<endl;
		cout<<"depth camera rows="<<_depth_camera_params.rows<<endl;
		cout<<"depth camera center x="<<_depth_camera_params.cx<<endl;
		cout<<"depth camera center y="<<_depth_camera_params.cy<<endl;
		cout<<"depth camera focus x="<<_depth_camera_params.fx<<endl;
		cout<<"depth camera focus y="<<_depth_camera_params.fy<<endl;

		cout<<"rgb camera cols="<<_rgb_camera_params.cols<<endl;
		cout<<"rgb camera rows="<<_rgb_camera_params.rows<<endl;
		cout<<"rgb camera center x="<<_rgb_camera_params.cx<<endl;
		cout<<"rgb camera center y="<<_rgb_camera_params.cy<<endl;
		cout<<"rgb camera focus x="<<_rgb_camera_params.fx<<endl;
		cout<<"rgb camera focus y="<<_rgb_camera_params.fy<<endl;

		cout<<"depth max truncation="<<_depth_prepocess_params.fMaxTrunc<<endl;
		cout<<"depth min truncation="<<_depth_prepocess_params.fMinTrunc<<endl;
		cout<<"bilinear filter sigma depth="<<_depth_prepocess_params.fSigmaDepth<<endl;
		cout<<"bilinear filter sigma pixel="<<_depth_prepocess_params.fSigmaPixel<<endl;


		cout<<"icp pyramid levels="<<_icp_params.nPyramidLevels<<endl;
		cout<<"icp threshold distance="<<_icp_params.fDistThres<<endl;
		cout<<"icp threshold pixel="<<_icp_params.fNormSinThres<<endl;
		cout<<"icp shaking distance="<<_icp_params.fDistShake<<endl;
		cout<<"icp shaking pixel="<<_icp_params.fAngleShake<<endl;

		cout<<"volume resolution="<<_volume_params.nResolution<<endl;
		cout<<"volume size meter="<<_volume_params.fVolumeMeterSize<<endl;
		cout<<"volume voxel max weight="<<_volume_params.fWeightMax<<endl;

		cout<<"volume integrate max distance="<<_integrate_params.fMaxIntegrateDist<<endl;
		cout<<"volume integrate sdf truncation="<<_integrate_params.fSdfTruncation<<endl;

		cout<<"raycasting increment="<<_raycast_params.fRayIncrement<<endl;

		cout<<"max mesh count="<<_marchingcube_params.uMaxTriangles<<endl;

		cout<<"rgbd data source filename="<<_io_params.rgbdReadFilename<<endl;
		cout<<"rgbd data record filename="<<_io_params.rgbdWriteFilename<<endl;
		cout<<"trajectory source filename="<<_io_params.trajReadFilename<<endl;
		cout<<"trajectory record filename="<<_io_params.trajWriteFilename<<endl;
		cout<<"mesh filename="<<_io_params.meshFilename<<endl;
	}
	SwitchParams _switch_params;
	CameraParams _rgb_camera_params;
	CameraParams _depth_camera_params;
	DepthPrepocessParams _depth_prepocess_params;
	IcpParams _icp_params;
	SDFTrackerParams _sdf_tracker_params;
	tsdfVolumeParams _volume_params;
	IntegrateParams _integrate_params;
	RayCasterParams _raycast_params;
	MarchingcubeParams _marchingcube_params;
	IOParams _io_params;

protected:
	AppParams(){}
	virtual ~AppParams(){}


};

#endif /* APPPARAMS_H_ */
