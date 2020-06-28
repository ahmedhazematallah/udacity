/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

// A helper function for main so that I can print all results for
// parts 7, 8, 9
int doMain(
    string detectorType, 
    string descriptorType,
    string matcherType,
    string selectorType)
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);

        // Check the buffer size
        if (dataBuffer.size() > dataBufferSize)
        {
            //cout << "New buffer size before erase: " << dataBuffer.size() << endl;
            
            // Erase elements from the beginning
            // Actually since we start by an empty buffer, so we need to remove only one element
            dataBuffer.erase(dataBuffer.begin());

            //cout << "New buffer size after erase: " << dataBuffer.size() << endl;
        }

        //// EOF STUDENT ASSIGNMENT
        cout << endl << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        //string detectorType = "HARRIS"/*"SHITOMASI"*/;

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        // Call detKeypointsModern with the detector type
        int tDetect = detKeypointsModern(keypoints, imgGray, detectorType, bVis);

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // loop on the keypoints and remove it if it is outside the box
            auto itr = keypoints.begin();
            while ( itr != keypoints.end())
            {
                // Check if the rectangle does not contain the key point
                if (vehicleRect.contains((*itr).pt) == false)
                {
                    // Update the iterator to the next one;
                    itr = keypoints.erase(itr);
                }
                else
                {
                    itr++;
                }                
            }
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        //string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        int tDesc = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            //string matcherType = "MAT_FLANN"/*"MAT_BF"*/;        // MAT_BF, MAT_FLANN
            string desType = "DES_BINARY"; // DES_BINARY, DES_HOG
		if ((descriptorType == "AKAZE") || (descriptorType == "SIFT"))
		{
			desType = "DES_HOG";
		}

            //string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, desType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // TASK.MP8
            cerr << "Img " << imgIndex 	<< ","
            	 << detectorType 		<< ","
				 << descriptorType 		<< ","
				 << matcherType 		<< ","
				 << selectorType 		<< ","
				 << keypoints.size() 	<< ","
    			 << matches.size() 		<< ","
				 << tDetect 			<< ","
				 << tDesc
				 << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                //cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    return 0;
}


/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    // All combinations
	vector<string> detectorTypes
        = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};

	vector<string> descriptorTypes
        = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

	vector<string> matcherTypes
        = {"MAT_BF", "MAT_FLANN"};

	// MP8 use BF only
	//vector<string> matcherTypes
	//        = {"MAT_BF"};

	vector<string> selectorTypes
        = {"SEL_NN", "SEL_KNN"};
    
	// MP.7, MP.8, MP.9
	cerr << "Img Id, Detector, Descriptor, Matcher, Selector, Num of Keypoints, Num of Matches, Time Detection, Time Descriptor" << endl;

    for (string detectorType : detectorTypes)
    {
        for (string descriptorType : descriptorTypes)
        {
        	// Skip the loop if the descriptor is AKAZE while the detector is not
        	// reference:https://docs.opencv.org/3.4/d8/d30/classcv_1_1AKAZE.html
        	//	"AKAZE descriptors can only be used with KAZE or AKAZE keypoints."
        	if ((descriptorType == "AKAZE") && (detectorType != "AKAZE"))
        	{
        		continue;
        	}

            for (string matcherType : matcherTypes)
            {
                for (string selectorType : selectorTypes)
                {
                    cout << endl 
                         << "<<<<<<<<<<<<<<< " 
                         << detectorType << ", "
                         << descriptorType << ", "
                         << matcherType << ", "
                         << selectorType << ", "
                         << " >>>>>>>>>>>>>>>" << endl;

                    doMain(detectorType, descriptorType, matcherType, selectorType);
                }
            }
        }
    }
    

    return 0;
}
