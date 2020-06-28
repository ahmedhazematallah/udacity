#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        if (descriptorType == "DES_HOG")
        {
        	normType = cv::NORM_L2;
        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {        
        matcher = cv::FlannBasedMatcher::create(/*indexParams, searchParams*/);

        if (descSource.type() != CV_32F)
        {
            descSource.convertTo(descSource, CV_32F);
        }

        if (descRef.type() != CV_32F)
        {
            descRef.convertTo(descRef, CV_32F);
        }
    }
    else
    {
        cout << "UNKNOWN MATCHER TYPE" << endl;
    }
    

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descSource, descRef, knnMatches, 2); // Finds the best match for each descriptor in desc1

        //cout << "-------------KNN -------------" << knnMatches.size() << endl;
        // Loop on all the matches pair
        for (auto itr = knnMatches.begin(); itr != knnMatches.end(); itr++)
        {
            // Each element is a vectorof size = 2 of DMatch
            if (( (*itr)[0].distance / (*itr)[1].distance) < 0.8) 
            {
                // distance ratio is less than 0.8
                matches.push_back((*itr)[0]);
                //cout << "SMALLER than 0.8: " << (*itr)[0].distance / (*itr)[1].distance << endl;
            }
            else
            {
                //cout << "LARGER than 0.8: " << (*itr)[0].distance / (*itr)[1].distance << endl;
            }
            
        }
    } else
    {
        cout << "UNKNOWN SELECTOR TYPE" << endl;
    }

    cout << "   Number of matched points: " << matches.size() << endl;
    
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
// BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
// Add return type for the time spent in extraction (in ms)
int descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType == "BRISK")
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if ( descriptorType == "BRIEF")
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if ( descriptorType == "ORB")
    {

        extractor = cv::ORB::create();
    }
    else if ( descriptorType == "FREAK")
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if ( descriptorType == "AKAZE")
    {
        extractor = cv::AKAZE::create();
    }
    else if ( descriptorType == "SIFT")
    {
        extractor = cv::SIFT::create();
    }
    else
    {
        cout << "******* UNKNOWN DESCRIPTOR TYPE: " << descriptorType << " *******" << endl;
    }
    

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

    return t * 1000;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    // double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    
    //t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    /*
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    */
}

void detKeypointsHarris(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img)
{
    // Create the destination matrix as zeros with the same size of the
    // input image
    cv::Mat dst, dstNormalized, dstScaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    int blockSize = 4;
    double k = 0.04;
    double maxOverlap = 0.0;
    int minResponse = 100;
    int apertSize = 3; // odd

    // Do Harris Detection using CV
    cv::cornerHarris(img, dst, blockSize, apertSize, k, cv::BORDER_DEFAULT);

    // Normalize the destination image
    cv::normalize(dst, dstNormalized, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // Convert Scale for visualization
    //convertScaleAbs(dstNormalized, dstScaled);

    // Loop on the rows
    for (int i = 0; i < dstNormalized.rows; i++)
    {
        // Loop on the columns
        for (int j = 0; j < dstNormalized.cols; j++)
        {
            // Read the current pixel
            int resp = dstNormalized.at<float>(i, j);

            // Create a new key point only if the response is greater than the min
            if (resp > minResponse)
            {
                // Create a new keypoint
                cv::KeyPoint keyPoint;
                keyPoint.pt = cv::Point2f(j, i);
                keyPoint.size = 2 * apertSize;
                keyPoint.response = resp;

                // Do NMS
                // Loop on all the current key points and check the ovverlap
                bool isOverlap = false;
                for (auto iter = keypoints.begin();
                     iter != keypoints.end();
                     iter++)
                {
                    double overlap = cv::KeyPoint::overlap(keyPoint, *iter);

                    if (overlap > maxOverlap)
                    {
                        // Set the overlap flag to avoid adding the keypoint
                        isOverlap = true;
                        // check which point has the bigger response
                        if (keyPoint.response > iter->response)
                        {
                            // Overwrite the found keypoint with the new one
                            (*iter) = keyPoint;
                            break; // stop looking for additional keypoints
                        }
                    }
                }

                // Add the keypoint only if no overlap was detected
                if (isOverlap == false)
                {
                    keypoints.push_back(keyPoint);
                }
            }
        }

    }
}


//   
void detKeypointsFAST(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img)
{
    cv::Ptr<cv::FeatureDetector> detector =
        cv::FastFeatureDetector::create(
            30, // threshold
            true, // NMS
            cv::FastFeatureDetector::TYPE_9_16
        );
    
    // do the detection
    detector->detect(img, keypoints);
}


void detKeypointsBRISK(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img)
{
    cv::Ptr<cv::BRISK> briskDetector = cv::BRISK::create();
     
    // do the detection
    briskDetector->detect(img, keypoints);
}

void detKeypointsORB(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img)
{
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
     
    // do the detection
    detector->detect(img, keypoints);
}

void detKeypointsAKAZI(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img)
{
    cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
     
    // do the detection
    detector->detect(img, keypoints);
}

void detKeypointsSIFT(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img)
{
    //cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
	cv::Ptr<cv::SIFT> detector = cv::SIFT::create();

    // do the detection
    detector->detect(img, keypoints);
}

//   //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
int detKeypointsModern(
    std::vector<cv::KeyPoint> &keypoints, 
    cv::Mat &img, 
    std::string detectorType, 
    bool bVis)
{
    // Record the current time (before detection)
    double t = (double)cv::getTickCount();

    if (detectorType == "SHITOMASI")
    {
        detKeypointsShiTomasi(keypoints, img);
    }
    else if (detectorType == "HARRIS")
    {
        detKeypointsHarris(keypoints, img);
    }
    else if (detectorType == "FAST")
    {
        detKeypointsFAST(keypoints, img);
    }
    else if (detectorType == "BRISK")
    {
        detKeypointsBRISK(keypoints, img);
    }
    else if (detectorType == "ORB")
    {
        detKeypointsORB(keypoints, img);
    }
    else if (detectorType == "AKAZE")
    {
        detKeypointsAKAZI(keypoints, img);
    }
    else if (detectorType == "SIFT")
    {
        detKeypointsSIFT(keypoints, img);
    }   
    else
    {
       cout << "Unknown Detector Type: " << detectorType << endl;
       return 0;
    }

    // Calculate the time spent in the detection
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return t *1000;
}
