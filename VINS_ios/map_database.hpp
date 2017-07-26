//
//  map_database.hpp
//  VINS_ios
//
//  Created by Tianu on 2017/7/24.
//  Copyright © 2017年 王荣志. All rights reserved.
//

#ifndef map_database_hpp
#define map_database_hpp

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "feature_manager.hpp"

using namespace cv;
using namespace Eigen;
using namespace std;


struct mapPoint
{
	Vector3d pt;
	//FeaturePerId fIdPt;
	bool state;
	int id;
};
class mapDatabase
{
public:
	mapDatabase();
	
	void fuseMap(FeaturePerId featureChain,Vector3d pt);
	void clear();
	
	~mapDatabase(){};
	
public:
	vector<mapPoint> mapPoints;
	int mapSize;
	
	
};
#endif /* map_database_hpp */
