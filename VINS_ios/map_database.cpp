//
//  map_database.cpp
//  VINS_ios
//
//  Created by Tianu on 2017/7/24.
//  Copyright © 2017年 王荣志. All rights reserved.
//

#include "map_database.hpp"

mapDatabase::mapDatabase():mapSize(0)
{
	
}

void mapDatabase::fuseMap(FeaturePerId featureChain, Vector3d pt){
	mapPoint fusedPoint;
	fusedPoint.pt=pt;
	//fusedPoint.fIdPt=featureChain;
	fusedPoint.id=featureChain.feature_id;
	fusedPoint.state=true;
	mapPoints.push_back(fusedPoint);
	mapSize++;
}

void mapDatabase::clear(){
	if(mapPoints.size()>0){
		mapPoints.clear();
		mapSize=0;
	}
}
