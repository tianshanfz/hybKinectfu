/*
 * SRKeyframeMan.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: hanyinbo
 */

#include "SRKeyframeMan.h"
#include"../cuda/DepthCamera.h"
#include "../cuda/CudaDeviceDataMan.h"
#include "../cuda/CudaWrappers.h"
#include "BlurEstimation.h"
#include "WeightDataCalculater.h"


void SRFrameBundle::updateNewDepthBranchAt(float2 screenpos,cv::Mat& reprojected,float depth,cv::Mat &new_weight,const cv::Mat& target_mat)const
{
	int rows=target_mat.rows;
	int cols=target_mat.cols;

	int2 screenpos_int=make_int2((int)(screenpos.x),(int)(screenpos.y));
	for(int x=screenpos_int.x;x<=screenpos_int.x+1;x++)
	for(int y=screenpos_int.y;y<=screenpos_int.y+1;y++)
	{

		if(x>=0&&x<cols&&y>=0&&y<rows)
		{
			float last_depth=reprojected.at<ushort>(y,x)/1000.0;
			float target_depth=target_mat.at<ushort>(y,x)/1000.0;
			float dist2=(x-screenpos.x)*(x-screenpos.x)+(y-screenpos.y)*(y-screenpos.y);

			if(new_weight.at<float>(y,x)==0||abs(last_depth-target_depth)>abs(depth-target_depth)&&abs(depth-target_depth)<0.01)
			{
				new_weight.at<float>(y,x)=1/depth/depth;//exp(-dist2);
				reprojected.at<ushort>(y,x)=depth*1000;

			}
		}
	}
}
SRDepthFrameBranch SRFrameBundle::updateNewDepthBranch(const DepthFrameData& frame_depth,const Mat44& camera_pose,const CameraParams& depth_camera_params)const
{//must be  _ref_camera_pos updated
	cv::Mat1f new_depth_weights=cv::Mat::zeros(frame_depth.mat().rows,frame_depth.mat().cols,CV_32F);
	cv::Mat1s new_reprojected_depths=cv::Mat1s::zeros(frame_depth.mat().rows,frame_depth.mat().cols);
	for(int row=0;row<frame_depth.mat().rows;row++)
	for(int col=0;col<frame_depth.mat().cols;col++)
	{
		float depth=frame_depth.mat().at<ushort>(row,col)/1000.0;
		if(depth==0)continue;
		float3 v=DepthCamera::depthToSkeleton(col,row,depth,depth_camera_params);
		float4 reprojected_v=_ref_camera_pos.getInverse()*camera_pose*make_float4(v.x,v.y,v.z,1);
		//cout<<reprojected_v.x<<reprojected_v.y<<reprojected_v.z<<endl;
		float2 screenpos_depth=DepthCamera::projectSkeletonToScreenfloat(make_float3(reprojected_v.x,reprojected_v.y,reprojected_v.z),depth_camera_params);
		if(_vec_depth_branch.size()==0)
		{
			new_reprojected_depths.at<ushort>(row,col)=frame_depth.mat().at<ushort>(row,col);
			new_depth_weights.at<float>(row,col)=1/depth/depth;
		}
		else
		{
			updateNewDepthBranchAt(screenpos_depth,new_reprojected_depths,reprojected_v.z,new_depth_weights,_vec_depth_branch[0].depths);
		}

	}
	DepthFrameData show;
	show.setFrameData(new_reprojected_depths,frame_depth.timeStamp());
	show.showFrame("reprojected depth");
	return {new_depth_weights,new_reprojected_depths};
}
uchar3 interpolateRGB(const cv::Mat3b& mat,float2 pos)
{
	int rows=mat.rows;
	int cols=mat.cols;

	if(pos.x<0||pos.x>=cols-1||pos.y<0||pos.y>=rows-1)return make_uchar3(0,0,0);

	int left=(int)(pos.x);
	int right=left+1;
	int top=(int)(pos.y);
	int bottom=top+1;


	uchar3 p00,p01,p10,p11;
	p00.x=mat.at<uchar>(top,left*3);p00.y=mat.at<uchar>(top,left*3+1);p00.z=mat.at<uchar>(top,left*3+2);

	p01.x=mat.at<uchar>(top,right*3);p01.y=mat.at<uchar>(top,right*3+1);p01.z=mat.at<uchar>(top,right*3+2);
	p10.x=mat.at<uchar>(bottom,left*3);p10.y=mat.at<uchar>(bottom,left*3+1);p10.z=mat.at<uchar>(bottom,left*3+2);
	p11.x=mat.at<uchar>(bottom,right*3);p11.y=mat.at<uchar>(bottom,right*3+1);p11.z=mat.at<uchar>(bottom,right*3+2);
	uchar3 p_top,p_bottom,ret;
	p_top.x=p00.x+(p01.x-p00.x)*(pos.x-left);
	p_top.y=p00.y+(p01.y-p00.y)*(pos.x-left);
	p_top.z=p00.z+(p01.z-p00.z)*(pos.x-left);
	p_bottom.x=p10.x+(p11.x-p10.x)*(pos.x-left);
	p_bottom.y=p10.y+(p11.y-p10.y)*(pos.x-left);
	p_bottom.z=p10.z+(p11.z-p10.z)*(pos.x-left);

	ret.x=p_top.x+(p_bottom.x-p_top.x)*(pos.y-top);
	ret.y=p_top.y+(p_bottom.y-p_top.y)*(pos.y-top);
	ret.z=p_top.z+(p_bottom.z-p_top.z)*(pos.y-top);
	return ret;
}
bool hasDepthDIscontinuities(cv::Mat depth_mat,float2 screenpos)
{
	int2 screenpos_int=make_int2((int)screenpos.x,(int)screenpos.y);
	if(screenpos_int.x>=depth_mat.cols||screenpos_int.x<0||screenpos_int.y>=depth_mat.rows||screenpos_int.y<0)
	{
		return true;
	}
	float depth=depth_mat.at<ushort>(screenpos_int.y,screenpos_int.x)/1000.0;
	if(depth==0)return true;
	for(int offset_x=-3;offset_x<=3;offset_x++)
	for(int offset_y=-3;offset_y<=3;offset_y++)
	{
		int neibour_x=screenpos.x+offset_x;
		int neibour_y=screenpos.y+offset_y;
		if(neibour_x>=depth_mat.cols||neibour_x<0||neibour_y>=depth_mat.rows||neibour_y<0)
		{
			return true;
		}
		float neibour_depth=depth_mat.at<ushort>(neibour_y,neibour_x)/1000.0;
		if(neibour_depth==0)return true;
		if(abs(neibour_depth-depth)>0.05)return true;

	}
	return false;
}
SRRGBFrameBranch SRFrameBundle::updateNewRGBBranch(bool is_ref,const cv::Mat& sr_depth,const ColorFrameData& frame_color,const Mat44& camera_pose,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params,int rate_rgb)const
{
	CameraParams rate_camera_params=rgb_camera_params*rate_rgb;
	cv::Mat1f new_rgb_weights=cv::Mat::zeros(rate_camera_params.rows,rate_camera_params.cols,CV_32F);
	cv::Mat3b new_reprojected_rgb=cv::Mat3b::zeros(rate_camera_params.rows,rate_camera_params.cols);
	BlurEstimation blur_estimater(frame_color.mat());
	float blur_score=blur_estimater.estimate();
	for(int row=0;row<new_reprojected_rgb.rows;row++)
	for(int col=0;col<new_reprojected_rgb.cols;col++)
	{
		float2 projected_screenpos;
		if(is_ref)
		{
			projected_screenpos.x=col*1.0/rate_rgb;
			projected_screenpos.y=row*1.0/rate_rgb;
		}
		else
		{
			float2 screenpos_depth;
			screenpos_depth.x=(col-rate_camera_params.cx)*depth_camera_params.fx/rate_camera_params.fx+depth_camera_params.cx;
			screenpos_depth.y=(row-rate_camera_params.cy)*depth_camera_params.fy/rate_camera_params.fy+depth_camera_params.cy;
			float depth=sr_depth.at<ushort>(screenpos_depth.y,screenpos_depth.x)/1000.0;
			if(depth==0)continue;
		//	if(hasDepthDIscontinuities(sr_depth,screenpos_depth))continue;
			float3 v=DepthCamera::depthToSkeleton(screenpos_depth.x,screenpos_depth.y,depth,depth_camera_params);
			float4 v_reprojected=camera_pose.getInverse()*_ref_camera_pos*make_float4(v.x,v.y,v.z,1);
			projected_screenpos=DepthCamera::projectSkeletonToScreenfloat(make_float3(v_reprojected.x,v_reprojected.y,v_reprojected.z),rgb_camera_params);

		}
		//float2 projected_screenpos_depth=DepthCamera::projectSkeletonToScreenfloat(make_float3(v_reprojected.x,v_reprojected.y,v_reprojected.z),depth_camera_params);
		//if(hasDepthDIscontinuities(frame_depth.mat(),projected_screenpos_depth))continue;

		uchar3 res= interpolateRGB(frame_color.mat(),projected_screenpos);

		if(isZero(res))
		{
			continue;
		}
		new_reprojected_rgb.at<uchar>(row,col*3)=res.x;
		new_reprojected_rgb.at<uchar>(row,col*3+1)=res.y;
		new_reprojected_rgb.at<uchar>(row,col*3+2)=res.z;

		new_rgb_weights.at<float>(row,col)=(1-blur_score);
		if(!is_ref)
		{
			new_rgb_weights.at<float>(row,col)*=_vec_depth_branch.back().weights.at<float>(row,col);
		}


	}
	cv::imshow("temp reprojected",new_reprojected_rgb);
	cv::waitKey();
	return {new_rgb_weights,new_reprojected_rgb};
}
void SRFrameBundle::reset()
{
	_vec_camera_pose.clear();
	_vec_depth_branch.clear();
	_vec_rgb_frames.clear();
}
void SRFrameBundle::addNewBranch(const DepthFrameData& frame_depth,const ColorFrameData& frame_color,const Mat44& camera_pose,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params)
{
	if(_vec_depth_branch.size()%_max_bundle_size==0)
	{
		_ref_camera_pos=camera_pose;
	}
	SRDepthFrameBranch depth_branch=updateNewDepthBranch(frame_depth,camera_pose,depth_camera_params);
	_vec_depth_branch.push_back(depth_branch);
	_vec_camera_pose.push_back(camera_pose);
	//SRRGBFrameBranch rgb_branch=updateNewRGBBranch(frame_depth,frame_color,camera_pose,depth_camera_params,rgb_camera_params);
	_vec_rgb_frames.push_back(frame_color);

}
cv::Mat SRFrameBundle::fusionDepthBranch()const
{
	cv::Mat1s new_sr_depths;
	if(_vec_depth_branch.size()==0)return new_sr_depths;
	int rows=_vec_depth_branch[0].depths.rows;
	int cols=_vec_depth_branch[0].depths.cols;
	new_sr_depths=cv::Mat1s::zeros(rows,cols);

	for(int row=0;row<rows;row++)
	for(int col=0;col<cols;col++)
	{
		float weight_sum=0;
		ushort sum=0;
		for(auto branch:_vec_depth_branch)
		{
			sum+=branch.depths.at<ushort>(row,col)*branch.weights.at<float>(row,col);
			weight_sum+=branch.weights.at<float>(row,col);

		}
		if(weight_sum>0)
		{
			new_sr_depths.at<ushort>(row,col)=sum/weight_sum;
		}
	}

	return new_sr_depths;
}
cv::Mat SRFrameBundle::fusionColorBranch(const cv::Mat& sr_depth,const CameraParams& depth_camera_params,const CameraParams& rgb_camera_params,int rate_rgb)const
{
	cv::Mat3b new_sr_rgb;
	if(_vec_rgb_frames.size()==0)return new_sr_rgb;
	std::vector<SRRGBFrameBranch> vec_rgb_branch;
	for(int i=0;i<_vec_rgb_frames.size();i++)
	{
		vec_rgb_branch.push_back(updateNewRGBBranch(i==0,sr_depth,_vec_rgb_frames[i],_vec_camera_pose[i], depth_camera_params,rgb_camera_params,rate_rgb));
	}
	int rows=rgb_camera_params.rows*rate_rgb;
	int cols=rgb_camera_params.cols*rate_rgb;
	new_sr_rgb=cv::Mat3b::zeros(rows,cols);

	for(int row=0;row<rows;row++)
	for(int col=0;col<cols;col++)
	{
		vector<float> weights;
		vector<uchar> vec_r,vec_g,vec_b;
		for(auto branch:vec_rgb_branch)
		{
			weights.push_back(branch.weights.at<float>(row,col));
			vec_r.push_back(branch.colors.at<uchar>(row,col*3));
			vec_g.push_back(branch.colors.at<uchar>(row,col*3+1));
			vec_b.push_back(branch.colors.at<uchar>(row,col*3+2));
		}

		new_sr_rgb.at<uchar>(row,col*3)=WeightDataCalculater<uchar>::calWeightMedian(vec_r,weights);
		new_sr_rgb.at<uchar>(row,col*3+1)=WeightDataCalculater<uchar>::calWeightMedian(vec_g,weights);
		new_sr_rgb.at<uchar>(row,col*3+2)=WeightDataCalculater<uchar>::calWeightMedian(vec_b,weights);
	}

	return new_sr_rgb;
}
/*void SRKeyframeMan::showSRFrame()
{
	_srframe_list.back().frame_depth.showFrame("sr frame");
	copySRFrameToGPU();
	cudaCalculateNewVertices(AppParams::instance()->_depth_camera_params);
	cudaCalculateNewNormals();
}
void SRKeyframeMan::copySRFrameToGPU()
{
	cv::Mat mat_depth=_srframe_list.back().frame_depth.mat();
	DepthfMap2D depth_data_cpu;
	int rows=mat_depth.rows,cols=mat_depth.cols;
	depth_data_cpu.create_cpu(cols,rows);
	depth_data_cpu.setZero();
	for(int row=0;row<rows;row++)
	{
		for(int col=0;col<cols;col++)
		{
			float v=mat_depth.at<unsigned short>(row,col)*0.001;//millimeter to meters
			depth_data_cpu.set_data(col,row,v);
		}
	}
	CudaDeviceDataMan::instance()->_raw_depth.copyFrom(depth_data_cpu);
	cudaTruncDepth(AppParams::instance()->_depth_prepocess_params.fMinTrunc,AppParams::instance()->_depth_prepocess_params.fMaxTrunc);
	cudaBiliearFilterDepth(AppParams::instance()->_depth_prepocess_params.fSigmaPixel,AppParams::instance()->_depth_prepocess_params.fSigmaDepth);
	depth_data_cpu.destroy();

}*/
