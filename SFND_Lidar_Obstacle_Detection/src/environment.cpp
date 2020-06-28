/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"


// Hyper parameters
#define FILTER_RES 0.3

// Region of interest
#define REGION_X_MIN    -10
#define REGION_X_MAX    30
#define REGION_Y_MIN    -7
#define REGION_Y_MAX    7
#define REGION_Z_MIN    -2
#define REGION_Z_MAX    3

// Segmentation Parameters
#define SEG_ITER    100
#define SEG_THRES   0.3

// Clustering Parameters
#define CLUSTER_MIN 10
#define CLUSTER_MAX 300
#define CLUSTER_TOL 0.53


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* myLidarPtr = new Lidar(cars, (double)0); 

    // Call scan method from the Lidar object to obtain a pointer to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr = myLidarPtr->scan();

    // Render the rays of the point cloud that is generated fom "scan"
    // renderRays(viewer, myLidarPtr->position , pointCloudPtr);
    renderPointCloud(viewer, pointCloudPtr, "MyPointCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointProcessorPtr = new ProcessPointClouds<pcl::PointXYZ>();  

    // Do the segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented;
    segmented = pointProcessorPtr->SegmentPlane(pointCloudPtr, SEG_ITER, 0.2);

    // Render both points
    //renderPointCloud(viewer, segmented.first, "ObstacleCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmented.second, "PlaneCloud", Color(0,1,0));

    // Do the clusteringfor the segmented obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters
        = pointProcessorPtr->Clustering(segmented.first, 1.0, 3, 30);

    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;

    // Iterate on the clusters to render them

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        renderPointCloud(viewer, 
                         cluster, 
                         "Obstcloud " + std::to_string(clusterId), 
                         colors[clusterId]);

        Box box = pointProcessorPtr->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        clusterId++;
    }

}


void cityBlock(
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ------------------------
    // 1. Filtering
    // ------------------------
    // Filter the point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud =
        pointProcessorI->FilterCloud(inputCloud, 
                                     FILTER_RES,
                                     Eigen::Vector4f(REGION_X_MIN, REGION_Y_MIN, REGION_Z_MIN, 1),
                                     Eigen::Vector4f(REGION_X_MAX, REGION_Y_MAX, REGION_Z_MAX, 1));
    
    // ------------------------
    //  2. Segmentation
    // ------------------------
    // Do the segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedRansac;
    
    // Use PCL for testing only
    /*
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPcl;
    segmentedPcl = pointProcessorI->SegmentPlane(filterCloud, SEG_ITER, SEG_THRES);
    std::cout << "PCL: " << segmentedPcl.second->points.size() << std::endl;*/
    
    segmentedRansac = pointProcessorI->SegmentPlaneRansac(filterCloud, SEG_ITER, SEG_THRES);
    //std::cout << "Ransac: " << segmented.second->points.size() << std::endl;

    // Render the plane    
    renderPointCloud(viewer, segmentedRansac.second, "PlaneCloud", Color(0,1,0));

    // ------------------------
    //  3. Clustering
    // ------------------------    
    // Do the clusteringfor the segmented obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters
        = pointProcessorI->ClusteringKdTree(segmentedRansac.first, 
                                            CLUSTER_TOL, 
                                            CLUSTER_MIN, 
                                            CLUSTER_MAX);

    // Use PCL for testing only
    /*
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClustersPcl
        = pointProcessorI->Clustering(segmentedPcl.first, 
                                      CLUSTER_TOL, 
                                      CLUSTER_MIN, 
                                      CLUSTER_MAX);*/

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    int clusterId = 0;

    // Iterate on the clusters to render them
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // Render the obstacle
        renderPointCloud(viewer, 
                         cluster, 
                         "Obstcloud " + std::to_string(clusterId), 
                         colors[clusterId % 3]);
        
        //std::cout << "Cluster Size: " << cluster->points.size() << std::endl;

        // Create a box around the obstacle and render it
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        // Move to the next cluster
        clusterId++;
    } 

}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS/*XY*/;
    initCamera(setAngle, viewer);
    

    // Use simple highway (Simulated LiDAR)
    //simpleHighway(viewer);

    // cityBlock (real LiDAR data)
    //cityBlock(viewer);
    // Create Point Processor
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI =
        new ProcessPointClouds<pcl::PointXYZI>(); 

    // Get a vector of paths to the files in the folder
    std::vector<boost::filesystem::path> stream =
        pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");

    // Create an iterator to point to the beginning
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Remove all shapes andpoint clouds of pervious files
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load the current PCD
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        // Call City Block for thecurrent file
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // Increment the iterator and reset it
        streamIterator++;
        if (streamIterator ==  stream.end())
        {
            streamIterator = stream.begin();
        }

        viewer->spinOnce ();
    }

    //while (!viewer->wasStopped ())
    //{
    //    viewer->spinOnce ();
    //} 
}