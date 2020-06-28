/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

void insertHelper(Node** node, std::vector<float> point, int id, int depth)
{
	if (*node == NULL)
	{
		*node = new Node(point, id);
	}
	else if (point[depth] < (*node)->point[depth])
	{
		insertHelper(&((*node)->left), point, id, (depth+1)%2);
	} 
	else
	{
		insertHelper(&((*node)->right), point, id, (depth+1)%2);
	}
}

void insertHelper3D(Node** node, std::vector<float> point, int id, int depth)
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

float measureDistance2D(std::vector<float> pt1, std::vector<float> pt2)
{
	float deltaX = (pt2[0] - pt1[0]);
	float deltaY = (pt2[1] - pt1[1]);

	return sqrt( deltaX * deltaX + deltaY * deltaY);
}

float measureDistance3D(std::vector<float> pt1, std::vector<float> pt2)
{
	float deltaX = (pt2[0] - pt1[0]);
	float deltaY = (pt2[1] - pt1[1]);
	float deltaZ = (pt2[2] - pt1[2]);

	return sqrt( deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
}

bool isInBox(std::vector<float> target, std::vector<float> test, float tol)
{	
	bool withinX = false; 
	bool withinY = false;

	if ( (test[0] >= (target[0] - tol) ) &&
	  	 (test[0] <= (target[0] + tol) ) ) 
	{
		withinX = true;	
	}
		
	if ( (test[1] >= (target[1] - tol) ) &&
	  	 (test[1] <= (target[1] + tol) ) ) 
	{
		withinY = true;		
	}
	
	return withinX && withinY;
}


bool isInBox3D(std::vector<float> target, std::vector<float> test, float tol)
{	
	bool withinX = false; 
	bool withinY = false;
	bool withinZ = false;

	if ( (test[0] >= (target[0] - tol) ) &&
	  	 (test[0] <= (target[0] + tol) ) ) 
	{
		withinX = true;	
	}
		
	if ( (test[1] >= (target[1] - tol) ) &&
	  	 (test[1] <= (target[1] + tol) ) ) 
	{
		withinY = true;		
	}

	if ( (test[2] >= (target[2] - tol) ) &&
	  	 (test[2] <= (target[2] + tol) ) ) 
	{
		withinZ = true;		
	}
	
	return withinX && withinY && withinZ;
}

void searchHelper(
	Node* node, 
	std::vector<float> target, 
	float distanceTol,
	int depth,
	std::vector<int>* idsPtr)
{
	if (node == NULL)
	{
		//std::cout << "	Reached a null node" << std::endl;
		return;
	}

	//std::cout << std::endl << "Calling search helper for Id: " << node->id << std::endl;

	// checkif the node is in the box of the target
	if (isInBox(target, node->point, distanceTol))
	{
		//std::cout << "	In box" << std::endl;

		if (measureDistance2D(target, node->point) <= distanceTol)
		{
			//std::cout << "-------Inserting-------" << std::endl;

			idsPtr->push_back(node->id);
		}
	} 
	
	//std::cout << "	Out of the box" << std::endl;

	if (target[depth]-distanceTol < node->point[depth])
	{
		//std::cout << "	Going left" << std::endl;

		searchHelper(node->left, target, distanceTol, (depth+1) % 2, idsPtr);
	} 

	if (target[depth]+distanceTol > node->point[depth])
	{
		//std::cout << "	Going right" << std::endl;
		searchHelper(node->right, target, distanceTol, (depth+1) % 2, idsPtr);
	}
	
}

void searchHelper3D(
	Node* node, 
	std::vector<float> target, 
	float distanceTol,
	int depth,
	std::vector<int>* idsPtr)
{
	if (node == NULL)
	{
		//std::cout << "	Reached a null node" << std::endl;
		return;
	}

	//std::cout << std::endl << "Calling search helper for Id: " << node->id << std::endl;

	// checkif the node is in the box of the target
	if (isInBox3D(target, node->point, distanceTol))
	{
		//std::cout << "	In box" << std::endl;

		if (measureDistance3D(target, node->point) <= distanceTol)
		{
			//std::cout << "-------Inserting-------" << std::endl;

			idsPtr->push_back(node->id);
		}
	} 
	
	//std::cout << "	Out of the box" << std::endl;

	if (target[depth]-distanceTol < node->point[depth])
	{
		//std::cout << "	Going left" << std::endl;

		searchHelper(node->left, target, distanceTol, (depth+1) % 3, idsPtr);
	} 

	if (target[depth]+distanceTol > node->point[depth])
	{
		//std::cout << "	Going right" << std::endl;
		searchHelper3D(node->right, target, distanceTol, (depth+1) % 3, idsPtr);
	}
	
}


struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper(root, target, distanceTol, 0, &ids);

		return ids;
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search3D(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		searchHelper3D(root, target, distanceTol, 0, &ids);

		return ids;
	}
};




