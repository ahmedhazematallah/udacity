
#ifndef RANSAC_H_
#define RANSAC_H_

#include <unordered_set>
#include "processPointClouds.h"


template<typename PointT>
class Ransac {
public:

    //constructor
    Ransac();

    //deconstructor
    ~Ransac();
    
    void RansacPlane(
		typename pcl::PointCloud<PointT>::Ptr cloud, 		
		int maxIterations, 
		float distanceTol,
		std::unordered_set<int>& inliersResult);

private:
    void fitPlane (
        PointT& point1, 
		PointT& point2, 
		PointT& point3, 
		float& A, 
		float& B, 
		float& C,
		float& D);

    double getDistanceToPlane (	
		float& A,
		float& B,
		float& C,
		float& D,
		PointT& point);
};

#endif