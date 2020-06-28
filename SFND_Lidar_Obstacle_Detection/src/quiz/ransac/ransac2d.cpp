/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}



void fitLine (
		pcl::PointXYZ point1, 
		pcl::PointXYZ point2, 
		float& A, 
		float& B, 
		float& C)
{
	float x1, x2, y1, y2;

	x1 = point1.x;
	y1 = point1.y;
	x2 = point2.x;
	y2 = point2.y;

	A = y1 - y2;

	B = x2 - x1;

	C = x1 * x2 - x2 * y1;
}

float getDistance
	(	
		float A,
		float B,
		float C,
		pcl::PointXYZ point
	)
{
	float x = point.x;
	float y = point.y;

	return std::fabs(A * x + B * y + C) / std::sqrt(A*A + B*B);
}


std::unordered_set<int> 
	Ransac
	(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		int maxIterations, 
		float distanceTol
	)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	int len = cloud->points.size();

	// TODO: Fill in this function

	int maxCountInliers = 0;

	// For max iterations 
	for (int iteration = 0; iteration < maxIterations; iteration++)
	{
		std::unordered_set<int> currentInliersResult;
		int currentCountInliers = 0;

		// Randomly sample subset and fit line
		int randIndex1 = rand() % len;
		int randIndex2 = rand() % len;

		float A,B,C;

		// Fit the line andobtain the coeff A, B, C
		fitLine(cloud->points[randIndex1], cloud->points[randIndex2], A, B, C);

		// Loop on all the points of the point cloud
		for (int pointInd = 0; pointInd < len; pointInd++)
		{
			// Measure distance between every point and fitted line
			float currentDistance = getDistance(A, B, C, cloud->points[pointInd]);
		
			// If distance is smaller than threshold count it as inlier
			if (currentDistance < distanceTol)
			{
				currentInliersResult.insert(pointInd);
				currentCountInliers++;
			}
		}

		if (currentCountInliers > maxCountInliers)
		{
			maxCountInliers = currentCountInliers;
			inliersResult.swap(currentInliersResult);
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	std::cout << "Inliers count: " << maxCountInliers <<std::endl;

	return inliersResult;

}


void fitPlane (
		pcl::PointXYZ point1, 
		pcl::PointXYZ point2, 
		pcl::PointXYZ point3, 
		float& A, 
		float& B, 
		float& C,
		float& D)
{
	float x1, x2, x3, y1, y2, y3, z1, z2, z3;

	x1 = point1.x;
	y1 = point1.y;
	z1 = point1.z;

	x2 = point2.x;
	y2 = point2.y;
	z2 = point2.z;

	x3 = point3.x;
	y3 = point3.y;
	z3 = point3.z;

	A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);

	B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);

	C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

	D = -(A * x1 + B * y1 + C * z1);
}

float getDistanceToPlane
	(	
		float A,
		float B,
		float C,
		float D,
		pcl::PointXYZ point
	)
{
	float x = point.x;
	float y = point.y;
	float z = point.z;

	return std::fabs(A * x + B * y + C * z + D) / std::sqrt(A*A + B*B + C*C);
}


std::unordered_set<int> 
	RansacPlane
	(
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		int maxIterations, 
		float distanceTol
	)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	int len = cloud->points.size();

	// TODO: Fill in this function

	int maxCountInliers = 0;

	// For max iterations 
	for (int iteration = 0; iteration < maxIterations; iteration++)
	{
		std::unordered_set<int> currentInliersResult;
		int currentCountInliers = 0;

		// Randomly sample subset and fit line
		int randIndex1 = rand() % len;
		int randIndex2 = rand() % len;
		int randIndex3 = rand() % len;

		float A, B, C, D;

		// Fit the plane and obtain the coeff A, B, C, D
		fitPlane(	cloud->points[randIndex1], 
					cloud->points[randIndex2],
					cloud->points[randIndex3], 
					A, B, C, D);

		// Loop on all the points of the point cloud
		for (int pointInd = 0; pointInd < len; pointInd++)
		{
			// Measure distance between every point and fitted line
			float currentDistance = getDistanceToPlane(A, B, C, D, cloud->points[pointInd]);
		
			// If distance is smaller than threshold count it as inlier
			if (currentDistance < distanceTol)
			{
				currentInliersResult.insert(pointInd);
				currentCountInliers++;
			}
		}

		if (currentCountInliers > maxCountInliers)
		{
			maxCountInliers = currentCountInliers;
			inliersResult.swap(currentInliersResult);
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	std::cout << "Inliers count: " << maxCountInliers <<std::endl;

	return inliersResult;

}



int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
