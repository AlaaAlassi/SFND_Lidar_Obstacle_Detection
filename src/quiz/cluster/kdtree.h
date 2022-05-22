/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

using namespace std;
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node **node, unsigned int depth, std::vector<float> point, int id)
	{
		int changeDimention = depth % 2;

		if (*node == NULL)
		{
			*node = new Node(point, id);
		}

		else if (point[changeDimention] < (*node)->point[changeDimention])
		{
			insertHelper(&(*node)->left, depth + 1, point, id);
		}
		else
		{
			insertHelper(&(*node)->right, depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		insertHelper(&root, 0, point, id);
	}

	void searchHelper(vector<float> target, Node *node, int depth, float distanceTolerance, vector<int> &ids)
	{
		if (node != NULL)
		{
			std::cout << "node coordinates x " << node->point[0] << " y " << node->point[1] << std::endl;
			float Px = target[0] - node->point[0];
			float Py = target[1] - node->point[1];
			bool isInsideTheBox = abs(Px) <= distanceTolerance && abs(Py) <= distanceTolerance;
			unsigned int depth = 0;
			int changeDimention = depth%2;
			/*std::cout << "root " << currentNode->point[0] << " " << currentNode->point[1] << std::endl;
			std::cout << "target " << target[0] << " "<< target[1] << std::endl;
			std::cout << "isInsideTheBox " << isInsideTheBox << std::endl;*/

			if (isInsideTheBox)
			{
				float d = sqrt(Px * Px + Py * Py);
				std::cout << "d " << d << std::endl;
				if (d <= distanceTolerance)
				{
					ids.push_back(node->id);
				}
			}

			//search the left branch
			auto rightArrow = (target[changeDimention]+distanceTolerance);
			bool searchRightBranch = rightArrow > node->point[changeDimention];
			if(searchRightBranch){
				cout << "searchRightBranch" << endl;
				searchHelper(target,node->right,depth+1,distanceTolerance,ids);
			}
			auto leftArrow = (target[changeDimention]-distanceTolerance);
			bool searchLeftBranch = leftArrow < node->point[changeDimention];
			if(searchLeftBranch){
				cout << "searchLeftBranch" << endl;
				searchHelper(target,node->left,depth+1,distanceTolerance,ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};
