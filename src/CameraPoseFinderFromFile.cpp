/*
 * CameraPoseFinderFromFile.cpp
 *
 *  Created on: May 30, 2016
 *      Author: hanyinbo
 */

#include "CameraPoseFinderFromFile.h"
#include "AppParams.h"
#include <Eigen/Eigen>
CameraPoseFinderFromFile::CameraPoseFinderFromFile(){
	// TODO Auto-generated constructor stub

}

CameraPoseFinderFromFile::~CameraPoseFinderFromFile() {
	// TODO Auto-generated destructor stub
	if(_trajfile_src.is_open())_trajfile_src.close();
}

bool CameraPoseFinderFromFile::initPoseFinder()
{
	_trajfile_src.open(AppParams::instance()->_io_params.trajReadFilename.c_str());
	string traj_line;
	if(false==_trajfile_src.is_open())return false;
	for(int i =0;i<3;i++)
	{
		getline(_trajfile_src,traj_line);
	}
	return true;
}
bool CameraPoseFinderFromFile::parseFrameFromFile(double target_timestamp,float3& t,float4& q,double &res_timestemp)
{
	string traj_line;
//	cout<<setprecision(15) << target_timestamp<<endl;
	double last_timestamp=0;
	float3 last_t;
	float4 last_q;
	//must be _trajfile_src.tellg.timestamp < target_timestamp
	while(!_trajfile_src.eof())
	{
		std::streampos file_pos = _trajfile_src.tellg();
		getline(_trajfile_src,traj_line);
		stringstream ss(traj_line);
		ss>>res_timestemp;
		ss>>t.x;ss>>t.y;ss>>t.z;
		ss>>q.x;ss>>q.y;ss>>q.z;ss>>q.w;
		if(res_timestemp>=target_timestamp)
		{
			if(res_timestemp-target_timestamp>target_timestamp-last_timestamp)
			{ //use last line data
				t=last_t;
				q=last_q;
				res_timestemp=last_timestamp;
				_trajfile_src.seekg(file_pos);//go back to last line for convenience of next frame

			}
			return true;
		}
		last_timestamp=res_timestemp;
		last_t=t;
		last_q=q;
	}
    return false;
}
bool CameraPoseFinderFromFile::estimateCameraPose(const DepthFrameData& depth_frame,const ColorFrameData& color_frame)
{

	float3 t;
	float4 q_temp;
	double timestamp;
	if(false==parseFrameFromFile(depth_frame.timeStamp(),t,q_temp,timestamp))return false;

	Eigen::Quaternion<float> q(q_temp.w,q_temp.x, q_temp.y, q_temp.z);
	Eigen::Matrix3f r = Eigen::Matrix3f(q);
	Eigen::Matrix3f r_trans=r.transpose();
	Mat44 file_transform=Mat44::getIdentity();
	Mat33 r_mat(r_trans.data());
	file_transform.setRotation(r_mat);
	file_transform.setTranslation(t);
	cout<<setprecision(15) <<depth_frame.timeStamp()<<" "<<color_frame.timeStamp()<<" "<<timestamp<<endl;
	if(depth_frame.frameId()==0)
	{
		_refer_transform=_pose*file_transform.getInverse();
		return true;
	}
	_pose=_refer_transform*file_transform;
	return true;
}
