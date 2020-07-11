
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

#include "configs.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
//void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &Prod)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        //Y = P_rect_xx * R_rect_xx * RT * X;
        Y = Prod * X;

        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
	//cout << "Entering clusterKptMatchesWithROI" << endl;

	if (kptMatches.size() == 0)	return;

	double distanceMean = 0.0;

	// 1- Find the match pairs that lies within the ROI

	//cout << "clusterKptMatchesWithROI before for loop" << endl;

    // Loop on all the keypoint match pairs
    for (auto matchItr = kptMatches.begin(); matchItr != kptMatches.end(); matchItr++)
    {
    	cv::DMatch matchPair = *matchItr;

    	int queryIdx = matchPair.queryIdx;
    	int trainIdx = matchPair.trainIdx;

    	// Extract the 2 keypoints
    	cv::KeyPoint currKp = kptsCurr.at(queryIdx);
    	cv::KeyPoint prevKp = kptsPrev.at(trainIdx);

    	// Extract the points of the keypoints
    	cv::Point2f prevPt = prevKp.pt;
    	cv::Point2f currPt = prevKp.pt;

    	//  Check if both points lie within the bounding box
    	if ( boundingBox.roi.contains(prevPt) &&
    		 boundingBox.roi.contains(currPt) )
    	{
    		// Add the current matched pair to the bounding box
    		boundingBox.kptMatches.push_back(matchPair);

    		// Accumulate the distance
    		distanceMean += matchPair.distance;
    	}
    }

    //cout << "clusterKptMatchesWithROI after for loop" << endl;

    // 2- Remove Outliers

    // Check to avoid division by zero (Found with SIFT, ORB descriptors)
    if (boundingBox.kptMatches.size() == 0)	return;

    // Calculate the mean
    distanceMean = distanceMean / boundingBox.kptMatches.size();

    //cout << "clusterKptMatchesWithROI before 2nd for loop" << endl;

    // Remove points that are far from the mean
    auto matchItr = boundingBox.kptMatches.begin();
    while (matchItr != boundingBox.kptMatches.end())
    {
    	//cout << "clusterKptMatchesWithROI inside 2nd for loop" << endl;

    	// I assume that if the absolute difference to the mean is greater than the mean i.e. 100% error
    	// then the matching point is considered as an outlier
    	if (fabs(matchItr->distance - distanceMean) > distanceMean)
		{
    		//cout << "clusterKptMatchesWithROI inside 2nd for loop if statement" << endl;
			//cout << "Erasing " << endl;
			boundingBox.kptMatches.erase(matchItr);
			// Don't increment the iterator since it is already incremented after erasing
		}
    	else
    	{
    		matchItr++;
    	}
    }
    //cout << "clusterKptMatchesWithROI after 2nd for loop" << endl;
    //int newSize = boundingBox.kptMatches.size();
    //cout << "Old: " << origSize << ", New: " << newSize << endl;
}

// Helper function to calculate the median of a vector
double getMedian(vector<double> values)
{
	// 1. Sort the vector
	std::sort(values.begin(), values.end());

	// 2. Check if the size is even or odd
	if (values.size() % 2)
	{
		// Odd -> Pick the middle element

		// Calculate the index of the middle element
		int index = (values.size() - 1) / 2;

		return values.at(index);
	}
	else
	{
		// Even -> Calculate the average of the 2 middle elements

		// Calculate the indices of the 2 middle elements
		int index1 = values.size() / 2;
		int index2 = index1 - 1;

		return (values.at(index1) + values.at(index2)) / 2;
	}
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios;

    // Loop on all matched key points twice and calculate the distance between
    // each two
    for (auto match1 = kptMatches.begin(); match1 != kptMatches.end()-1; match1++)
    { // outer kpt. loop

        // Extract the 2 matched 2 keypoints across the 2 frames
        cv::KeyPoint kp1Prev = kptsPrev.at(match1->trainIdx);
        cv::KeyPoint kp1Curr = kptsCurr.at(match1->queryIdx);

        // Start one point ahead
        for (auto match2 = kptMatches.begin() + 1; match2 != kptMatches.end(); match2++)
        {
            // Extract the 2 matched 2 keypoints across the 2 frames
            cv::KeyPoint kp2Prev = kptsPrev.at(match2->trainIdx);
            cv::KeyPoint kp2Curr = kptsCurr.at(match2->queryIdx);

            // Measure the current and previous distances between the 2 chosen points
            double currDistance = cv::norm(kp1Curr.pt - kp2Curr.pt);
            double prevDistance = cv::norm(kp1Prev.pt - kp2Prev.pt);

            //cout << "	currDistance = " << currDistance;
            //cout << "	prevDistance = " << prevDistance << endl;

            // Measure the distance ratio between current and previous
            // Skip points that are too near to each others
            if (currDistance >= MIN_DISTANCE)
            {
            	// Check that the denominator is not zero and also not too small
            	if (prevDistance > std::numeric_limits<double>::epsilon())
            	{
            		double distRatio = currDistance / prevDistance;
            		distRatios.push_back(distRatio);
            	}
            }
        }
    }

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    double medianDistanceRatio = getMedian(distRatios);

    double delta_t = 1 / frameRate;

    TTC = -delta_t / (1 - medianDistanceRatio);

    //cerr << "	TTC Camera = " << TTC << " seconds." << endl;
}


double findMinX(std::vector<LidarPoint> &lidarPoints, double& xMean)
{
	// 1- Calculate the mean
	xMean = 0.0;

	for (auto pt : lidarPoints)
	{
		xMean += pt.x;
	}

	xMean = xMean / lidarPoints.size();

	// 2- Find the min value of X
	double xMeanThreshold = THRESHOLD_X_MEAN; // 10cm

	double xMin = 1e9;

    for(auto it=lidarPoints.begin(); it!=lidarPoints.end(); ++it) {

    	// Skip the points whose X is different than the mean by the threshold value
    	if (fabs(it->x - xMean) > xMeanThreshold)
    	{
    		// Skipping
    		//cout << "Skipping " << it->x << ", while X mean = " << xMean << endl;
    		continue;
    	}

    	//cout << it->x << ", ";

    	xMin = it->x < xMin ?  it->x : xMin;
    }
    return xMin;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
	double dT = 1/ frameRate;

    // Find the prev min X
    double xMeanPrev;
    double xMinPrev = findMinX(lidarPointsPrev, xMeanPrev);

#if FP5
    cout << "Prev xMin, xMean, diff = " << xMinPrev << ", " << xMeanPrev << ", " << xMinPrev - xMeanPrev << endl;
#endif

    // Find the current min X
    double xMeanCurr;
    double xMinCurr = findMinX(lidarPointsCurr, xMeanCurr);

#if FP5
    cout << "Curr xMin, xMean, diff = " << xMinCurr << ", " << xMeanCurr << ", " << xMinCurr - xMeanCurr << endl;
#endif

#if FP5
    cout << "If we used mean, TTC Lidar = " << xMeanCurr * dT / (xMeanPrev - xMeanCurr) << endl;
#endif
    // compute TTC from both measurements
    TTC = xMinCurr * dT / (xMinPrev - xMinCurr);

//    cerr << "	TTC Lidar = " << TTC << " seconds." << endl;
}


// Helper function to find the box in which a point is located
// Note:
//    - Stops on the first found box
//    - If no box is found, the function returns -1
int findBB(cv::Point2f &pt, vector<BoundingBox> &boundingBoxes)
{
	// Loop on all the bounding boxes
	for (int i = 0; i < boundingBoxes.size(); i++)
	{
		if (boundingBoxes.at(i).roi.contains(pt))
		{
			return i;
		}
	}
	// Return -1 if no box is found
	return -1;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //cout << "Prev frame has the bounding boxes of size: " << prevFrame.boundingBoxes.size() << endl;
    //cout << "Curr frame has the bounding boxes of size: " << currFrame.boundingBoxes.size() << endl;

    int numPrevBB = prevFrame.boundingBoxes.size();
    int numCurrBB = currFrame.boundingBoxes.size();

    // Use a 2-D matrix to count the number of matches between BB across all the matches
    int matchCount[numPrevBB][numCurrBB] = {0};

    // Loop on all the keypoint match pairs
    for (auto matchItr = matches.begin(); matchItr != matches.end(); matchItr++)
    {
    	cv::DMatch matchPair = *matchItr;

    	int trainIdx = matchPair.trainIdx;
    	int queryIdx = matchPair.queryIdx;

    	//cout << trainIdx << ", " << queryIdx << endl;

    	// Extract the 2 keypoints
    	cv::KeyPoint prevKp = prevFrame.keypoints.at(trainIdx);
    	cv::KeyPoint currKp = currFrame.keypoints.at(queryIdx);

    	// Extract the points of the keypoints
    	cv::Point2f prevPt = prevKp.pt;
    	cv::Point2f currPt = currKp.pt;

    	// Find the bounding box of the previous point
    	int prevBB = findBB(prevPt, prevFrame.boundingBoxes);

    	// Find the bounding box of the current point
    	int currBB = findBB(currPt, currFrame.boundingBoxes);

    	if (prevBB >=0 && currBB >=0)
    	{
    		matchCount[prevBB][currBB]++;
    	}
    	else
    	{
    		//std::cerr << "At least one of the matched points is not found in a bounding box\n";
    	}
    }

    // Loop on the matchCount array to find the best match.
    // For each row, I choose the column with the max count
    for (int i = 0; i < numPrevBB; i++)
    {
    	// Find the max count
    	int maxValue = -1;
    	int maxIndex = -1;
    	for (int j=0; j < numCurrBB; j++)
    	{
    		// Find the 1st max
    		if (matchCount[i][j] > maxValue)
    		{
    			maxValue = matchCount[i][j];
    			maxIndex = j;
    		}
    	}
    	// Check if a count greater than zero was found
    	if (maxValue > 0)
    	{
    		// Check of we shall store the box id instead
    		bbBestMatches.insert({prevFrame.boundingBoxes[i].boxID, currFrame.boundingBoxes[maxIndex].boxID});
    	}
    }

    //cout<< "	The number of BB best matches found is: " << bbBestMatches.size() << endl;
}
