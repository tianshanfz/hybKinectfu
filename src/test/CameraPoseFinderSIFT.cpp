/*
 * CameraPosFinderSIFT.cpp
 *
 *  Created on: Jun 1, 2016
 *      Author: hanyinbo
 */
#if    0 //unsed
#include "CameraPoseFinderSIFT.h"
#include "../keyframeMan.h"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
using namespace cv;
using namespace pcl;
CameraPoseFinderSIFT::CameraPoseFinderSIFT() {
	// TODO Auto-generated constructor stub

}

CameraPoseFinderSIFT::~CameraPoseFinderSIFT() {
	// TODO Auto-generated destructor stub
}

bool CameraPoseFinderSIFT::initPoseFinder()
{
	return true;
}
bool FeatureMatch(const cv::Mat& img_1,
	const cv::Mat& img_2,
	vector< cv::KeyPoint > &keypoints_1,
	vector< cv::KeyPoint > &keypoints_2,
	vector< cv::DMatch > &good_matches
	)
{
	if (!img_1.data || !img_2.data)
	{
		cout << "Error reading images" << endl;
		return false;
	}

	// detect
	cv::SurfFeatureDetector detector(400);

	detector.detect(img_1, keypoints_1);
	detector.detect(img_2, keypoints_2);
	// descriptor
	cv::SurfDescriptorExtractor extractor;
	cv::Mat descriptors_1, descriptors_2;
	extractor.compute(img_1, keypoints_1, descriptors_1);
	extractor.compute(img_2, keypoints_2, descriptors_2);
	// match
	cv::FlannBasedMatcher matcher;
	vector< cv::DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);

	double max_dist = 0; double min_dist = 100.00;
	for (int i = 0; i< descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	for (int i = 0; i< descriptors_1.rows; i++)
	{
		if (matches[i].distance <= max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}
	cv::Mat img_matches;
	cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	cv::imshow("good match", img_matches);
	return true;
}
bool EstimateMotion(const cv::Mat& depth_cur,const cv::Mat& depth_pre, const CameraParams& depth_camera_params,
		const cv::Mat& rgb_cur, const cv::Mat& rgb_pre,
	const vector< KeyPoint >& keypoints1,
	const vector< KeyPoint >& keypoints2,
	const vector< DMatch >& goodmatches,
	Eigen::Matrix4f &rt
	)
{
	PointCloud< PointXYZRGBA >::Ptr cloudin(new PointCloud< PointXYZRGBA >);
	PointCloud< PointXYZRGBA >::Ptr cloudout(new PointCloud< PointXYZRGBA >);
	cout << keypoints1.size()<<" "<<keypoints2.size();


	for (vector< DMatch > ::const_iterator it = goodmatches.begin(); it != goodmatches.end(); ++it)
	{
		Point current_pt = keypoints1 [it->queryIdx].pt;
		Point old_pt = keypoints2[it->trainIdx].pt;
		if (current_pt.x >= 640 || current_pt.x < 0 || current_pt.y >= 480 || current_pt.y < 0)
		{
			continue;
		}

		if (old_pt.x >= 640 || old_pt.x < 0 || old_pt.y >= 480 || old_pt.y < 0)
		{
			continue;
		}
		float depthCur = 0.001*depth_cur.at<unsigned short>(current_pt.y,current_pt.x);
		if (depthCur == 0)
		{
			continue;
		}

		float depthPre =0.001*depth_pre.at<unsigned short>(old_pt.y,old_pt.x);
		if (depthPre == 0)
		{
			continue;
		}

		PointXYZRGBA newPointCur, newPointPre;
		newPointCur.z = depthCur;
		newPointCur.x=newPointCur.z*(current_pt.x-depth_camera_params.cx)/depth_camera_params.fx;
		newPointCur.y=newPointCur.z*(current_pt.y-depth_camera_params.cy)/depth_camera_params.fy;
		newPointCur.r = rgb_cur.at<cv::Vec3b>(current_pt.y,current_pt.x)[0];
		newPointCur.g= rgb_cur.at<cv::Vec3b>(current_pt.y,current_pt.x)[1];
		newPointCur.b = rgb_cur.at<cv::Vec3b>(current_pt.y,current_pt.x)[2];

		newPointPre.z=depthPre;
		newPointPre.x=newPointPre.z*(old_pt.x-depth_camera_params.cx)/depth_camera_params.fx;
		newPointPre.y=newPointPre.z*(old_pt.y-depth_camera_params.cy)/depth_camera_params.fy;
		newPointPre.r = rgb_pre.at<cv::Vec3b>(old_pt.y,old_pt.x)[0];
		newPointPre.g =rgb_pre.at<cv::Vec3b>(old_pt.y,old_pt.x)[1];
		newPointPre.b =rgb_pre.at<cv::Vec3b>(old_pt.y,old_pt.x)[2];

	//	cout << current_pt << old_pt<<endl;
	//	cout << newPointCur << newPointPre << endl;
		cloudin->push_back(newPointCur);
		cloudout->push_back(newPointPre);

	}
	cout << "start icp :" << endl;
	IterativeClosestPoint< PointXYZRGBA, PointXYZRGBA > icp;
	icp.setInputSource(cloudin);
	icp.setInputTarget(cloudout);
	icp.setMaxCorrespondenceDistance(0.08); // 8cm
	icp.setMaximumIterations(50);
	PointCloud< PointXYZRGBA > Final;

	icp.align(Final);
	if (icp.hasConverged())
	{
		cout << "icp has converged: " << icp.hasConverged() << endl;
		rt = icp.getFinalTransformation();
	//	writePointsToMatFile(cloudin, cloudout, rt);
	//	printf("write to v1 v2 txt...."); getchar();
	//	cout << rt << endl;
		return true;
	}
	else
		return false;

}
bool CameraPoseFinderSIFT::estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)
{
	if(depth_frame.frameId()==0)
	{
		return true;
	}
	KeyFrame lastkeyframe=KeyframeMan::instance()->getLastKeyFrame();
	vector< cv::KeyPoint > keypoints_1;
	vector< cv::KeyPoint > keypoints_2;
	vector< cv::DMatch >good_matches;
	Eigen::Matrix4f rt;
	//double t0=clock();
	FeatureMatch(color_frame.mat(), lastkeyframe.frame_rgb.mat(), keypoints_1, keypoints_2, good_matches);
	//cout<<"feature match cost"<<(clock()-t0)/1000.0<<"ms"<<endl;
	if(false== EstimateMotion(depth_frame.mat(),lastkeyframe.frame_depth.mat(),AppParams::instance()->_depth_camera_params,
			color_frame.mat(), lastkeyframe.frame_rgb.mat(),keypoints_1,keypoints_2,good_matches,rt))
	{
		return false;
	}
	Eigen::Matrix4f tt =rt.transpose();
	_pose =Mat44(tt.data())*lastkeyframe.cameraPose;
	//_pose.print();
	//getchar();
	return true;
}
#endif
