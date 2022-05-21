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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;
	Node *currentNode = NULL;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

void insertHelper(Node **node,unsigned int depth,std::vector<float> point, int id)
   {
	int changeDimention = depth%2;

      if(*node == NULL)
      {
        *node = new Node(point,id);
      }

      else if(point[changeDimention] < (*node)->point[changeDimention])
      {
        insertHelper(&(*node)->left,depth+1,point,id);
      }
      else
      {
        insertHelper(&(*node)->right,depth+1,point,id);
      }
   }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		insertHelper(&root,0,point,id);

	}

	void searchHelper(){

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		if(currentNode == NULL){
			currentNode = root;
		}
		std::vector<int> ids;
		float Px = target[0] - currentNode->point[0];
		float Py = target[1] - currentNode->point[1];
		bool isInsideTheBox = abs(Px) <= distanceTol && abs(Py) <= distanceTol;
		unsigned int depth =0;
        bool isOnRight = target[0] + distanceTol > 0;
		std::cout << "root " << currentNode->point[0] << " " << currentNode->point[1] << std::endl;
		std::cout << "target " << target[0] << " "<< target[1] << std::endl;
		std::cout << "isInsideTheBox " << isInsideTheBox << std::endl;
		if(isInsideTheBox){
			float d = sqrt(Px*Px + Py*Py);
			std::cout << "d " << d << std::endl;
			if(d <= distanceTol){
				ids.push_back(currentNode->id);
			}


		}

		return ids;
	}
	

};




