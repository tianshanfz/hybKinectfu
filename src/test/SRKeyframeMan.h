/*
 * SRKeyframeMan.h
 *
 *  Created on: Jun 27, 2016
 *      Author: hanyinbo
 */

#ifndef SRKEYFRAMEMAN_H_
#define SRKEYFRAMEMAN_H_
#include"../FrameData.h"
#include"../utils/cpu_include.h"
#include"../cuda/Mat.h"
#include "../keyframeMan.h"
#include "../AppParams.h"

struct SRDepthFrameBranch
{
	cv::Mat1f weights;
	cv::Mat1s depths;
	SRDepthFrameBranch(cv::Mat1f w,cv::Mat1s d):weights(w),depths(d){}
};
struct SRRGBFrameBranch
{
	cv::Mat1f weights;
	cv::Mat3b colors;
	SRRGBFrameBranch(cv::Mat1f w,cv::Mat3b c):weights(w),colors(c){}
};
class SRFrameBundle
{
public:
	SRFrameBundle(int bundle_size):_max_bundle_size(bundle_size){}
	void addNewBranch(const DepthFrameData& frame_depth,const ColorFrameData& frame_color,const Mat44& camera_pose,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params);
	cv::Mat  fusionDepthBranch() const;
	cv::Mat fusionColorBranch(const cv::Mat &sr_depths,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params,int rate_rgb)const;
	bool isFull()const{return _vec_depth_branch.size()==_max_bundle_size;}
	const Mat44& getRefCameraPos()const{return _ref_camera_pos;}
	void reset();
protected:
	SRDepthFrameBranch updateNewDepthBranch(const DepthFrameData& frame_depth,const Mat44& camera_pose,const CameraParams& depth_camera_params)const;
	void updateNewDepthBranchAt(float2 screenpos,cv::Mat& reprojected,float depth,cv::Mat &new_weight,const cv::Mat& target_mat)const;
	SRRGBFrameBranch updateNewRGBBranch(bool is_ref,const cv::Mat& sr_depth,const ColorFrameData& frame_color,const Mat44& camera_pose,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params,int rate_rgb)const;
	

	const unsigned _max_bundle_size;
	Mat44 _ref_camera_pos;
	vector<SRDepthFrameBranch> _vec_depth_branch;
	vector<ColorFrameData> _vec_rgb_frames;
	vector<Mat44> _vec_camera_pose;
};
class SRKeyframeMan {
public:
	SRKeyframeMan(int bundle_size):_cur_bundle(bundle_size){}
	void updateFrame(const DepthFrameData& frame_depth,const ColorFrameData& frame_color,const Mat44& camera_pose,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params)
	{
		_cur_bundle.addNewBranch(frame_depth, frame_color,camera_pose, depth_camera_params,rgb_camera_params);
		if(_cur_bundle.isFull())
		{
			DepthFrameData sr_depth_frame;
			cv::Mat sr_depths=_cur_bundle.fusionDepthBranch();
			sr_depth_frame.setFrameData(sr_depths,frame_depth.timeStamp());
			sr_depth_frame.showFrame("sr_depth");

			ColorFrameData sr_color_frame;
			cv::Mat sr_rgb=_cur_bundle.fusionColorBranch(sr_depths,depth_camera_params,rgb_camera_params,1);
			sr_color_frame.setFrameData(sr_rgb,frame_color.timeStamp());
			sr_color_frame.showFrame("sr_rgb");

			_cur_bundle.reset();
		}
	}
	virtual ~SRKeyframeMan(){}
protected:
	SRFrameBundle _cur_bundle;
};
/*class SRKeyframeMan {
public:
	SRKeyframeMan(int bundle_size):_cur_bundle(bundle_size){}

	virtual ~SRKeyframeMan(){}
	void showSRFrame();
	void updateFrame(const DepthFrameData& frame_depth,const ColorFrameData& frame_color,const Mat44& camera_pose,const CameraParams& depth_camera_params);
protected:
	void copySRFrameToGPU();
	void calNewSRFrame(const DepthFrameData& frame_depth,const ColorFrameData& frame_color,const Mat44& camera_pose);
	void fusionToSRFrame(const DepthFrameData& frame_depth,
						 const Mat44& origin_pose,
						 const CameraParams& depth_camera_params);

	SRFrameBundle _cur_bundle;
	vector<KeyFrame> _srframe_list;
};*/

#endif /* SRKEYFRAMEMAN_H_ */
