#include "ransac.h"


//constructor:
template<typename PointT>
Ransac<PointT>::Ransac() {}


//de-constructor:
template<typename PointT>
Ransac<PointT>::~Ransac() {}


// Fit a plane through 3 points and return the coefficients of the plane
template<typename PointT>
void Ransac<PointT>::fitPlane (
		PointT& point1, 
		PointT& point2, 
		PointT& point3, 
		float& A, 
		float& B, 
		float& C,
		float& D)
{
	double x1, x2, x3, y1, y2, y3, z1, z2, z3;

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

// Calculate the distance between a Plan and a point
template<typename PointT>
double Ransac<PointT>::getDistanceToPlane
	(	
		float& A,
		float& B,
		float& C,
		float& D,
		PointT& point
	)
{
	return std::fabs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A*A + B*B + C*C);
}

template<typename PointT>
void Ransac<PointT>::RansacPlane(
		typename pcl::PointCloud<PointT>::Ptr cloud, 		
		int maxIterations, 
		float distanceTol,
		std::unordered_set<int>& inliersResult)
{
	//srand(time(NULL));
	unsigned int seed = time(NULL);
	//std::cout << "Seed: " << seed << std::endl;
	srand(seed);
	
	int len = cloud->points.size();

	int maxCountInliers = 0;

	// For max iterations 
	for (int iteration = 0; iteration < maxIterations; iteration++)
	{
		std::unordered_set<int> currentInliersResult;
		int currentCountInliers = 0;

		// Get 3 random unique points
		std::set<int> randIndex;
		while (randIndex.size() < 3)
		{
			int randNum = rand() % len;
			randIndex.insert(randNum);
			//std::cout << "  Random (" << iteration << "): " << randNum << std::endl;
		}

		float A, B, C, D;

		std::set<int>::iterator randItr = randIndex.begin();

		// Fit the plane through the 3 points and obtain the coeff A, B, C, D
		fitPlane(	cloud->points[*(randItr++)], 
					cloud->points[*(randItr++)],
					cloud->points[*(randItr++)], 
					A, B, C, D);

		// Loop on all the points of the point cloud
		for (int pointInd = 0; pointInd < len; pointInd++)
		{
			// Measure distance between every point and fitted line
			float currentDistance = getDistanceToPlane(A, B, C, D, cloud->points[pointInd]);
		
			// If distance is smaller than threshold count it as inlier
			if (currentDistance <= distanceTol)
			{
				// Insert the index in the set
				currentInliersResult.insert(pointInd);

				// Increase the current count of inliers
				currentCountInliers++;
			}
		}

		//std::cout << "	currentCountInliers = " << currentCountInliers << std::endl;

		// Check if we have more than the previous max
		if (currentCountInliers > maxCountInliers)
		{
			// Update the maximum
			maxCountInliers = currentCountInliers;

			// Update the result set with the current set
			inliersResult = currentInliersResult;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	//std::cout << "Inliers count: " << maxCountInliers <<std::endl;
}


