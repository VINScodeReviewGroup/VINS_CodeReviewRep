//
//  feature_tracker.cpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "feature_tracker.hpp"

int FeatureTracker::n_id = 0;
FeatureTracker::FeatureTracker()
:mask{ROW, COL, CV_8UC1},update_finished{false},img_cnt{0}
{
    printf("init ok\n");
}
/*********************************************************tools function for feature tracker start*****************************************************/
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

 template <typename T>
void reduceVector(vector<T> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
        max_min_pts tmp;
        tmp.min = p;
        tmp.max = p;
        parallax_cnt.push_back(tmp);
    }
}

void FeatureTracker::setMask()
{
    mask.setTo(255);
    
    // prefer to keep features that are tracked for long time
	
    vector<pair<pair<int, max_min_pts>, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(make_pair(track_cnt[i], parallax_cnt[i]), make_pair(forw_pts[i], ids[i])));
    //根据特征点被跟踪的帧数目由大到小进行排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &a, const pair<pair<int, max_min_pts>, pair<cv::Point2f, int>> &b)
         {
             return a.first.first > b.first.first;
         });
    
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();
    parallax_cnt.clear();
    //对相应已检测特征点位置设置mask
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        //if(true)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first.first);
            parallax_cnt.push_back(it.first.second);
			//设置mask，不是画图...
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    //for (auto &it: pre_pts)
    //{
    //    cv::circle(mask, it, MIN_DIST, 0, -1);
    //}
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        vector<uchar> status;

        cv::findFundamentalMat(pre_pts, forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        reduceVector(pre_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(parallax_cnt, status);
    }
}

/*********************************************************tools function for feature tracker ending*****************************************************/


void FeatureTracker::readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, vector<Point2f> &good_pts, vector<double> &track_len)
{
	//特征点检测后的图像
	result = _img;
	//处理首次接收图像情况;每次有前一帧，当前帧，后一帧共三帧
	if(forw_img.empty())
		pre_img = cur_img = forw_img = _img;
	else
		forw_img = _img;
	
	forw_pts.clear();
	
	//track
	{
		if(cur_pts.size()>0)
		{
			vector<uchar> status;
			vector<float> err;
			
			//TS(time_track);
			//KLT光流法检测特帧点并跟踪
			calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
			//TE(time_track);
			//跟踪的角点是否在图像内
			for (int i = 0; i < int(forw_pts.size()); i++)
				if (status[i] && !inBorder(forw_pts[i]))
					status[i] = 0;
			//去除光流法跟踪中跟踪不好的角点，pre_pts,cur_pts,forw_pts始终保持相同大小，且相同索引对应一组跟踪角点
			reduceVector(pre_pts, status);
			reduceVector(cur_pts, status);
			reduceVector(forw_pts, status);
			reduceVector(ids, status);
			reduceVector(track_cnt, status);
			reduceVector(parallax_cnt, status);
			
			if(img_cnt!=0)
			{
				for (int i = 0; i< forw_pts.size(); i++)
				{
					//cv::line(result, pre_pts[i], forw_pts[i], cvScalar(0), 3, 8, 0);
					good_pts.push_back(forw_pts[i]);
					//不断更新每组跟踪角点位移最大的两点用于计算视差，但这种方法只适合角点短时间内朝某个方向移动，移动方向最好不要出现转弯
					if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
					{
						parallax_cnt[i].min = forw_pts[i];
					}
					else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
					{
						parallax_cnt[i].max = forw_pts[i];
					}
					double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
					track_len.push_back(std::min(1.0, 1.0 * parallax/30));
				}
			}
		}
	}
	
	//detect
	{
		//img_cnt=（img_cnt+1)%3;
		if(img_cnt==0)
		{
			//根据前一帧和后一帧之间的由光流法跟踪匹配得到的匹配点对计算F矩阵，ransanc算法，去除部分outlier
			rejectWithF();
			
			for (int i = 0; i< forw_pts.size(); i++)
			{
				good_pts.push_back(forw_pts[i]);
				if(forw_pts[i].x < parallax_cnt[i].min.x || forw_pts[i].y < parallax_cnt[i].min.y)
				{
					parallax_cnt[i].min = forw_pts[i];
				}
				else if(forw_pts[i].x > parallax_cnt[i].max.x || forw_pts[i].y > parallax_cnt[i].max.y)
				{
					parallax_cnt[i].max = forw_pts[i];
				}
				double parallax = (cv::norm(parallax_cnt[i].max - parallax_cnt[i].min) < 2.0? 0: cv::norm(parallax_cnt[i].max - parallax_cnt[i].min));
				track_len.push_back(std::min(1.0, 1.0 * parallax/50));
			}
			//更新每组角点能被跟踪的帧数,n是引用
			for (auto &n : track_cnt)
				n++;
			
			//根据track_cnt由大到小排序已跟踪角点组；设置mask，将已检测出角点位置一定半径内设置为不再检测角点区域
			setMask();
			int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
			
			//如果目前被跟踪角点组数小于MAX_CNT,则尝试检测更多新角点
			if(n_max_cnt>0)
			{
				n_pts.clear();
				TS(time_goodfeature);
				//goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.10, MIN_DIST, mask, 3, false, 0.04);
				goodFeaturesToTrack(forw_img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
				TE(time_goodfeature);
			}
			else
			{
				n_pts.clear();
			}
			//将n_pts复制到forw_pts，ids=-1，track_cnt=1,max_min_pts=(p,p)
			addPoints();
			//printf("features num after detect: %d\n",static_cast<int>(forw_pts.size()));
			pre_img = forw_img;
			pre_pts = forw_pts;
			//draw
			for (int i = 0; i < n_pts.size(); i++)
			{
				good_pts.push_back(n_pts[i]);
				track_len.push_back(0);
			}
			//result = mask;
			
		}
		cur_img = forw_img;
		cur_pts = forw_pts;
	}
	if(img_cnt == 0)
	{
		//update id and msg
		image_msg.clear();
		int num_new = 0;
		//更新新添加的角点的id
		for (unsigned int i = 0;; i++)
		{
			bool completed = false;
			completed |= updateID(i);
			if (!completed)
				break;
		}
		//将所有角点反投影取单位深度存储在image_msg中
		for(int i = 0; i<ids.size(); i++)
		{
			double x = (cur_pts[i].x - PX)/FOCUS_LENGTH_X;
			double y = (cur_pts[i].y - PY)/FOCUS_LENGTH_Y;
			double z = 1.0;
			image_msg[(ids[i])] = (Vector3d(x, y, z));
		}
	}
	//finished and tell solver the data is ok
	update_finished = true;
}
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}
