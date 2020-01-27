#include "render/render.h"

template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT setPoint, int setId)
	:	point(setPoint), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

	void doInsert(Node<PointT>*& node, PointT point, int id, int depth)
	{
		if (node == NULL)
		{
			node = new Node<PointT>(point, id);
		}
		else
		{
			int pointIndex = depth % 3;
			if (point.data[pointIndex] <= node->point.data[pointIndex])
				doInsert(node->left, point, id, depth + 1);
			else
				doInsert(node->right, point, id, depth + 1);
		}
	}

	void insert(PointT point, int id)
	{
		doInsert(root, point, id, 0);
	}

	// returns a point that describes the location of point in relation to target
	PointT* boxCompare(PointT target, PointT point)
	{
		PointT* comp = new PointT;
		comp->x = point.x - target.x;
		comp->y = point.y - target.y;
		comp->z = point.z - target.z;
		return comp;
	}

	float distance(PointT point1, PointT point2)
	{
		float x1 = point1.x;
		float y1 = point1.y;
		float z1 = point1.z;
		float x2 = point2.x;
		float y2 = point2.y;
		float z2 = point2.z;
		return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
	}

	void doSearch(Node<PointT>* node, std::vector<int>& ids, PointT target, float distanceTol, uint depth)
	{
		if (node == NULL)
			return;

		uint splitIndex = depth % 3;

		PointT point = node->point;
		PointT* comp = boxCompare(target, point);

		if (fabs(comp->x) <= distanceTol && fabs(comp->y) <= distanceTol && fabs(comp->z) && distance(target, point) <= distanceTol)
			ids.push_back(node->id);

		if ((target.data[splitIndex] - distanceTol) < point.data[splitIndex])
			doSearch(node->left, ids, target, distanceTol, depth + 1);

		if ((target.data[splitIndex] + distanceTol) > point.data[splitIndex])
			doSearch(node->right, ids, target, distanceTol, depth + 1);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		doSearch(root, ids, target, distanceTol, 0);
		return ids;
	}
};




