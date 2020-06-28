/* \author Aaron Brown */
// Quiz on implementing kd tree

#ifndef KDTREE_H_
#define KDTREE_H_

//#include "../../render/render.h"
#include <vector>
#include <math.h>
#include <string.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert3D(std::vector<float>& point, int id);

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(std::vector<float>& target, float& distanceTol);

	std::vector<std::vector<int>> euclideanCluster(
		std::vector<std::vector<float>>& points, 
		float distanceTol,
		int min,
		int max);

private:
	void proximity(
		std::vector<std::vector<float>>& points, 	
		float distanceTol, 
		int pointId,
		std::vector<int>& 	cluster,
		std::vector<bool>& 	isProcessed);

};

#endif


