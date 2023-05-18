// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>

struct Node
{
	pcl::PointXYZI point;
	int id;
	Node* left;
	Node* right;

	Node(pcl::PointXYZI arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, pcl::PointXYZI point, int id)
	{
		if(*node == NULL)
			*node = new Node(point, id);
		else
		{
			if (depth%3==0)
			{
				if (point.x < ((*node)->point.x))
					insertHelper(&((*node)->left), depth+1, point, id);
				else 
					insertHelper(&((*node)->right), depth+1, point, id);
			}
			else if (depth%3==1)
			{
				if (point.y < ((*node)->point.y))
					insertHelper(&((*node)->left), depth+1, point, id);
				else 
					insertHelper(&((*node)->right), depth+1, point, id);
			}
			else if (depth%3 ==2)
			{
				if (point.z < ((*node)->point.z))
					insertHelper(&((*node)->left), depth+1, point, id);
				else 
					insertHelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void searchHelper(pcl::PointXYZI target, Node** node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if((*node)!=NULL)
		{ 
			float px = (*node)->point.x;
			float py = (*node)->point.y;
			float pz = (*node)->point.z;
			
			float tx = target.x;
			float ty = target.y;
			float tz = target.z;
			
			float xminDist = tx - distanceTol;
			float xmaxDist = tx + distanceTol;
			float yminDist = ty - distanceTol;
			float ymaxDist = ty + distanceTol;
			float zminDist = tz - distanceTol;
			float zmaxDist = tz + distanceTol;
			
			float Dist = sqrt(pow((px-tx),2) + pow((py-ty),2) + pow((pz-tz),2));
			
			if((px>=xminDist&&px<=xmaxDist) && (py>=yminDist&&py<=ymaxDist) && (pz>=zminDist&&pz<=zmaxDist))
			{
				if(Dist <= distanceTol)
					ids.push_back((*node)->id);
			}
			
			if (depth%3==0)
			{
				if(xminDist < px)
					searchHelper(target, &((*node)->left), depth+1, distanceTol, ids);
				else if (xmaxDist > px)
					searchHelper(target, &((*node)->right), depth+1, distanceTol, ids);
			}
			else if (depth%3==1)
			{
				if(yminDist < py)
					searchHelper(target, &((*node)->left), depth+1, distanceTol, ids);
				else if (ymaxDist > py)
					searchHelper(target, &((*node)->right), depth+1, distanceTol, ids);
			}
			else if (depth%3==2)
			{
				if(zminDist < pz)
					searchHelper(target, &((*node)->left), depth+1, distanceTol, ids);
				else if (zmaxDist > pz)
					searchHelper(target, &((*node)->right), depth+1, distanceTol, ids);
			}
		}
	}

	void insert(pcl::PointXYZI point, int id)
	{
		insertHelper(&root, 0, point, id);
	}


	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, &root, 0, distanceTol, ids);
		return ids;
	}		

};

template<typename PointT>
class ProcessPointClouds {
public:

	//constructor
  ProcessPointClouds();
  //deconstructor
  ~ProcessPointClouds();

  void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

  typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>  SegmentPlane3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);
	
	void Proximity(int indice, typename pcl::PointCloud<PointT>::Ptr cloud_id, std::vector<int>& cluster_id, std::vector<bool> processed, KdTree* tree, float distanceTol);
	
	std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minClusterSize, int maxClusterSize);

  std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

	std::unordered_set<int> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);


  Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

  void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

  typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

  std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};




#endif /* PROCESSPOINTCLOUDS_H_ */
