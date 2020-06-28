// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "ransac.h"
#include "ransac.cpp"

#include "kdtree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, 
    Eigen::Vector4f minPoint, 
    Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;

    // -------------------------
    //  1. Vox Grid Filtering
    // -------------------------
    // Set input cloud
    vg.setInputCloud(cloud);

    // Set the leaf size
    vg.setLeafSize(filterRes, filterRes, filterRes);

    // Do the filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>());
    vg.filter(*cloudFiltered);

    // ------------------------------------
    //  2. Filter the region box using crop
    // ------------------------------------
    // Create a crop box    
    pcl::CropBox<PointT> region(true);

    // Set the min and max of the region    
    region.setMin(minPoint);
    region.setMax(maxPoint);

    // Set the input
    region.setInputCloud(cloudFiltered);

    // Do the filtering
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>());
    region.filter(*cloudRegion);
  
    // ------------------------------------
    //  3. Remove the points from the roof
    // ------------------------------------
    // ------------------------------------
    //  3. a. Find the indices of points in the box of the roof
    // ------------------------------------
    // Create a crop box    
    pcl::CropBox<PointT> roof(true);

    // Set the min and max of the region    
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));

    // Set the input
    roof.setInputCloud(cloudRegion);

    // Find the indices
    std::vector<int> indices;
    roof.filter(indices);
    //std::cout<< "Indices in the roof " << indices.size() << std::endl;

    // ------------------------------------
    //  3. b. Remove the points using extract
    // ------------------------------------
    // Create point indices from the indices that we obtained
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int pointIndex : indices)
    {
        inliers->indices.push_back(pointIndex);
    }

    // Create Extract Object (Negative)
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SeparateClouds
    (
        pcl::PointIndices::Ptr inliers, // Indices of points that are in the plane
        typename pcl::PointCloud<PointT>::Ptr cloud // Whole point cloud
    ) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>() );
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>() );

    // Fill the plane cloud by the content of the indices
    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // Create the extractor object to filter the plane points
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    
    // Filter the plan points out of the original cloud in order to obtain
    // the obstacle cloud.
    extract.filter(*obstacleCloud);
    
    // Retrun the pair of Plane & Obstacles
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SegmentPlane
    (
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceThreshold
    )
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.

    // 1. Create segmentation object
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices };
    pcl::ModelCoefficients * coeffPtr = new pcl::ModelCoefficients();
    pcl::SACSegmentation<PointT> seg;
    
    // 2.Configure the segmentation object
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // 3. Pass the input
    seg.setInputCloud(cloud);

    // 4. Do the segmentation
    seg.segment(*inliers, *coeffPtr);

    // 5. Check the size of inliers
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not find plane points" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation Pcl took " << elapsedTime.count() 
              << " milliseconds and found " << inliers->indices.size() << " inliers." << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::SegmentPlaneRansac
    (
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        int maxIterations, 
        float distanceThreshold
    )
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // 1. Create segmentation object
    
    Ransac<PointT> *myRansac = new Ransac<PointT>();

    // Do segmentation using RANSAC
    std::unordered_set<int> indices;
    myRansac->RansacPlane(cloud, maxIterations, distanceThreshold, indices);
	
    // Copy the indices to PointIndices
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices() };
    for (int index : indices)
    {
        inliers->indices.push_back(index);
    }


    // 5. Check the size of inliers
    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not find plane points" << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation Ransac took " << elapsedTime.count() 
              << " milliseconds and found " << inliers->indices.size() <<" inliers." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
        segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    // Create KdTree Object
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    
    tree->setInputCloud(cloud);

    // Create Euclidean cluster extraction
    pcl::EuclideanClusterExtraction<PointT> ec;
    
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // Extract the indices of the clusters in a vector
    std::vector<pcl::PointIndices> clusterIndices;
    ec.extract(clusterIndices);

    // Loop on the resulting vector to extract the points of each cluster
    for (std::vector<pcl::PointIndices>::const_iterator clusterIt = clusterIndices.begin();
         clusterIt != clusterIndices.end();
         clusterIt++)
    {
        // Create a point cloud for the cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        // Fill the point cloud
        // Loop on the indices
        for (std::vector<int>::const_iterator pointIt = clusterIt->indices.begin();
             pointIt != clusterIt->indices.end();
             pointIt++)
        {
            cloudCluster->points.push_back(cloud->points[*pointIt]);
        }

        // Push the point cloud to the return of the function
        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering PCL took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
    ProcessPointClouds<PointT>::ClusteringKdTree(
        typename pcl::PointCloud<PointT>::Ptr cloud, 
        float clusterTolerance, 
        int minSize, 
        int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Create KdTree Object
    KdTree* tree = new KdTree();

    std::vector<std::vector<float>> pointVector;

    // Fill the tree with the point cloud
    for (int i=0; i<cloud->points.size(); i++)
    {
        // Create a vector of float for the point
        std::vector<float> point({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    	
        pointVector.push_back(point);

        // Insert the point into the tree
        tree->insert3D(point,i);
    }
    
    auto midTime1 = std::chrono::steady_clock::now();

    // Do Euclidean Clustering
    std::vector<std::vector<int>> clusterIndices = 
        tree->euclideanCluster(pointVector, clusterTolerance, minSize, maxSize);

    auto midTime2 = std::chrono::steady_clock::now();

    // Loop on the resulting vector to extract the points of each cluster
    for (std::vector<std::vector<int>>::const_iterator clusterIt = clusterIndices.begin();
         clusterIt != clusterIndices.end();
         clusterIt++)
    {
        // Create a point cloud for the cluster
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        // Fill the point cloud
        // Loop on the indices
        for (std::vector<int>::const_iterator pointIt = clusterIt->begin();
             pointIt != clusterIt->end();
             pointIt++)
        {
            cloudCluster->points.push_back(cloud->points[*pointIt]);
        }

        // Push the point cloud to the return of the function
        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    auto part1 = std::chrono::duration_cast<std::chrono::milliseconds>(midTime1 - startTime);
    auto part2 = std::chrono::duration_cast<std::chrono::milliseconds>(midTime2 - midTime1);
    auto part3 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - midTime2);
    std::cout << "clustering KdTree took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    //std::cout << "  " << part1.count() << " ms & " 
    //                  << part2.count() << " ms & " 
    //                  << part3.count() << " ms" << std::endl;
    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << std::endl<< "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}