/* \author Ahmed Atallah */

#include "kdtree.h"
#include <iostream>


void insertHelper(Node** node, std::vector<float>& point, int id, int depth)
{
	if (*node == NULL)
	{
		*node = new Node(point, id);
	}
	else if (point[depth] < (*node)->point[depth])
	{
		insertHelper(&((*node)->left), point, id, (depth+1)%3);
	} 
	else
	{
		insertHelper(&((*node)->right), point, id, (depth+1)%3);
	}
}


inline float measureDistance3D(std::vector<float>& pt1, std::vector<float>& pt2)
{
	float deltaX = (pt2[0] - pt1[0]);
	float deltaY = (pt2[1] - pt1[1]);
	float deltaZ = (pt2[2] - pt1[2]);

	return sqrt( deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
}


inline bool isInBox(std::vector<float>& target, std::vector<float>& test, float& tol)
{	
	if ( (test[0] >= (target[0] - tol) ) &&
	  	 (test[0] <= (target[0] + tol) )) 
	{
		if ((test[1] >= (target[1] - tol) ) &&
	  	 	(test[1] <= (target[1] + tol) ) )
		{
			if ((test[2] >= (target[2] - tol) ) &&
	  	 		(test[2] <= (target[2] + tol) ) )
			{
				return true;
			}
		}
	}
		
	return false;
}


void searchHelper3D(
	Node* node, 
	std::vector<float>& target, 
	float distanceTol,
	int depth,
	std::vector<int>* idsPtr)
{
	if (node == NULL)
	{
		//std::cout << "	Reached a null node" << std::endl;
		return;
	}
	
	// checkif the node is in the box of the target
	if (isInBox(target, node->point, distanceTol))
	{
		//std::cout << "	In box" << std::endl;

		if (measureDistance3D(target, node->point) <= distanceTol)
		{
			//std::cout << "-------Inserting-------" << std::endl;
			//std::cout << measureDistance3D(target, node->point) << std::endl;
			idsPtr->push_back(node->id);
		}
	} 
	
	//std::cout << "	Out of the box" << std::endl;

	if (target[depth]-distanceTol < node->point[depth])
	{
		//std::cout << "	Going left" << std::endl;

		searchHelper3D(node->left, target, distanceTol, (depth+1) % 3, idsPtr);
	} 

	if (target[depth]+distanceTol > node->point[depth])
	{
		//std::cout << "	Going right" << std::endl;
		searchHelper3D(node->right, target, distanceTol, (depth+1) % 3, idsPtr);
	}
}




void KdTree::insert3D(std::vector<float>& point, int id)
{
	// the function should create a new node and place correctly with in the root 

	insertHelper(&root, point, id, 0);
}

// return a list of point ids in the tree that are within distance of target
std::vector<int> KdTree::search3D(std::vector<float>& target, float& distanceTol)
{
	std::vector<int> ids;

	searchHelper3D(root, target, distanceTol, 0, &ids);

	return ids;
}

void KdTree::proximity(
	std::vector<std::vector<float>>& points, 	
	float distanceTol, 
	int pointId,
	std::vector<int>& 	cluster,
	std::vector<bool>& 	isProcessed)
{
	// Mark the point as processed
	isProcessed[pointId] = true;

	// Add the point to the cluster
	cluster.push_back(pointId);

	//std::cout << "Adding point and its nearby (x,y,z): " 
	//	      << points[pointId][0] << ", "
	//		  << points[pointId][1] << ", "
	//		  << points[pointId][2] << std::endl;


	// Get nearby points
	std::vector<int> nearbyPoints = search3D(points[pointId], distanceTol);

	// Loop on nearby points
	for (int id : nearbyPoints)
	{
		// if point is not processed
		if (isProcessed[id] == false)
		{
			// Call proximity again
			proximity(points, distanceTol, id, cluster, isProcessed);
		}			
	}
}

std::vector<std::vector<int>> KdTree::euclideanCluster(
	std::vector<std::vector<float>>& points, 
	float distanceTol,
	int min,
	int max)
{
	std::vector<std::vector<int>> clusters;

	// Create a vector of bool to indicate that a point is processed
	std::vector<bool> isProcessed(points.size(), false);

	// Iterate on all points
	for (int i = 0; i < points.size(); i++)
	{
		// if the point is not processed
		if (isProcessed[i] == false)
		{
			// Create cluster
			std::vector<int> newCluster;

			// Call Proximity
			proximity(points, distanceTol, i, newCluster, isProcessed);

			int newClusterSize = newCluster.size();			
			if ((newClusterSize >= min) && (newClusterSize <= max))
			{
				// Add the cluster to the clusters
				clusters.push_back(newCluster);
			}			
		}
	}
 
	return clusters;
}




