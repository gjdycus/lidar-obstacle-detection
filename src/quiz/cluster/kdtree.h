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

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void doInsert(Node* &node, std::vector<float> point, int id, int depth)
	{
		if (node == NULL)
		{
			node = new Node(point, id);
		}
		else
		{
			int pointIndex = depth % 2;
			if (point[pointIndex] <= node->point[pointIndex])
				doInsert(node->left, point, id, depth + 1);
			else
				doInsert(node->right, point, id, depth + 1);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		doInsert(root, point, id, 0);
	}

	// Returns a point that describes the location of point in relation to target
	std::vector<float> boxCompare(std::vector<float> target, std::vector<float> point)
	{
		std::vector<float> comp;
		comp.push_back(point[0] - target[0]);
		comp.push_back(point[1] - target[1]);
		return comp;
	}

	float distance(std::vector<float> point1, std::vector<float> point2)
	{
		float x1 = point1[0];
		float y1 = point1[1];
		float x2 = point2[0];
		float y2 = point2[1];
		return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
	}

	void doSearch(Node* node, std::vector<int> &ids, std::vector<float> target, float distanceTol, uint depth)
	{
		if (node == NULL)
			return;

		uint splitIndex = depth % 2;

		std::vector<float> point = node->point;
		std::vector<float> comp = boxCompare(target, point);

		if (fabs(comp[0]) <= distanceTol && fabs(comp[1]) <= distanceTol && distance(target, point) <= distanceTol)
			ids.push_back(node->id);

		if ((target[splitIndex] - distanceTol) < point[splitIndex])
			doSearch(node->left, ids, target, distanceTol, depth + 1);

		if ((target[splitIndex] + distanceTol) > point[splitIndex])
			doSearch(node->right, ids, target, distanceTol, depth + 1);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		doSearch(root, ids, target, distanceTol, 0);
		return ids;
	}
};




