struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* left;
	
	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node* root;
	
	KdTree()
	:	root(NULL)
	{}
	
	void insertHelper(Node *&node, uint level, PointT point, int id)
	{
		uint index = level%3;
		
		if(node == NULL)
		{
			std::vector<float> v_point(point.data, point.data+3);
			node = new Node(v_point, id);
		}
		else if(point.data[index] < node->point[index])
		{
			insertHelper(node->left, level+1, point, id);
		}
		else
		{
			insertHelper(node->right, level+1, point, id);
		}
	}
	
	void searchHelper(Node *&node, uint level, std::vector<int> *ids, PointT target, float distanceTol)
	{
		uint id = level%3;
		if(node!=NULL)
		{