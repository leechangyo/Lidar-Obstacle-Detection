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

	// 
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		helper_function(&root, 0, point, id);

	}

	// TODO : Desing Binary Tree
	// node pointer was actually what root was to begin with and now we are doing memory address of this pointer node
	// so "Node** node" is just memory address that's pointg at our node that we are currently on the tree
	void helper_function(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id); // assign node in the tree;
		}
		else
		{
			// calculate current dim((why "%2"? to compare x, or y)
			// how to traveler tree? it is depending on the depth
			// 2D case here, depth is even or odd.
			// if cd is zero, see x value, if not , see y value.
			uint cd = depth % 2;
			if(point[cd]<((*node)->point[cd]))
			{
				helper_function(&((*node)->left), depth+1, point,id);
			}
			else
			{
				helper_function(&((*node)->right), depth+1, point, id);
			}
		}
		
	}

	// TODO : design Search algorithms
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		KDTree_search_hepler(root, 0 , ids , target, distanceTol);
		return ids;
	}
	// TODO : KD Search(nearneibor) algorithm(fine near point corresponding to target point)
	void KDTree_search_hepler(Node* node, int depth, std::vector<int>& ids, std::vector<float> target ,float distanceTo)
	{
		// see if that node is within that target box or not
		if(node!=NULL)
		{
			// box contraint
			if((node->point[0]>=(target[0]-distanceTo)) && (node->point[0]<=(target[0]+distanceTo)) && (node->point[1] >= (target[1]-distanceTo)) && (node->point[1]<=(target[1]+distanceTo)))
			{
				// euclidian distance
				float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0]) + (node->point[1]-target[1])*(node->point[1]-target[1]));
			
				if(distance <= distanceTo)
				{
					ids.push_back(node->id);
				}
			}
			// checking box accross boundary(x-y comparison)
			if((target[depth%2]-distanceTo)<node->point[depth%2]) // if it is , target is left box region
				KDTree_search_hepler(node->left, depth+1, ids, target, distanceTo);
			if((target[depth%2]+distanceTo)>node->point[depth%2]) // if it is , target then, right box region.
				KDTree_search_hepler(node->right, depth+1, ids, target, distanceTo);
		}

	}



};




