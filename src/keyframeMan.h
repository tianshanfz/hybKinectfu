/*
 * keyframeMan.h
 *
 *  Created on: Jun 1, 2016
 *      Author: hanyinbo
 */

#ifndef KEYFRAMEMAN_H_
#define KEYFRAMEMAN_H_

#include"FrameData.h"
#include"utils/cpu_include.h"
#include"cuda/Mat.h"
struct KeyFrame
{
	ColorFrameData frame_rgb;
	DepthFrameData frame_depth;
	Mat44		camera_pose;
	KeyFrame(const DepthFrameData &newdepth,const ColorFrameData &new_color,const Mat44& new_camera_pose):frame_depth(newdepth),frame_rgb(new_color),camera_pose(new_camera_pose){}
	KeyFrame(const KeyFrame &frame)
	{
		frame.frame_depth.copyTo(frame_depth);
		frame.frame_rgb.copyTo(frame_rgb);
		camera_pose=frame.camera_pose;
	}
};
class KeyframeMan {
public:

	static KeyframeMan* instance()
	{
		static KeyframeMan value;
		return &value;
	}

	void addNewKeyFrame(const DepthFrameData &depth,const ColorFrameData &rgb,const Mat44& camera_pose)
	{
		KeyFrame newframe(depth,rgb,camera_pose);
		if(_list.size()==_max_size)
		{
			_list.pop_front();
		}
		_list.push_back(newframe);
	}
	void writeToFile(const string& index_filename)
	{
		int dir_idx=index_filename.find_last_of('/');
		string dir=index_filename.substr(0,dir_idx==-1?index_filename.size():dir_idx);
		std::ofstream idx_file(index_filename.c_str());
		int i=0;
		for(auto &keyframe:_list)
		{
			stringstream sfilename;
			sfilename<<"keyframe_rgb"<<i<<".png";
			string filename;
			sfilename>>filename;
			sfilename.clear();
			keyframe.frame_rgb.saveToFile(dir+'/'+filename);
			idx_file<<filename<<" ";

			sfilename<<"keyframe_depth"<<i<<".png";
			sfilename>>filename;
			sfilename.clear();
			keyframe.frame_depth.saveToFile(dir+'/'+filename);
			idx_file<<filename<<endl;

			idx_file<<keyframe.camera_pose;
			i++;
		}
	}
	KeyFrame& getLastKeyFrame(){return _list.back();}
	list<KeyFrame> getFrameList()const{return _list;}
private:
	KeyframeMan():_max_size(50){}
	virtual ~KeyframeMan(){}
	list<KeyFrame> _list;
	int _max_size;
};

#endif /* KEYFRAMEMAN_H_ */
