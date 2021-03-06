// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*cloud);

    pcl::CropBox<PointT> cropBox;
    cropBox.setInputCloud(cloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.filter(*cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, Box> ProcessPointClouds<PointT>::RemoveRegionFromCloud(typename pcl::PointCloud<PointT>::Ptr cloud, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    pcl::CropBox<PointT> cropBox;
    cropBox.setInputCloud(cloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setNegative(true);
    cropBox.filter(*cloud);

    Box box;
    box.x_min = minPoint.x();
    box.y_min = minPoint.y();
    box.z_min = minPoint.z();
    box.x_max = maxPoint.x();
    box.y_max = maxPoint.y();
    box.z_max = maxPoint.z();

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "region removal took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, Box> res(cloud, box);
    return res;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointIndices::Ptr& inliers, int maxIterations, double distanceTol)
{
	srand(time(NULL));

	// For max iterations
	while (maxIterations--) {
		// Randomly sample subset and fit line
		int cloudSize = cloud->size();
		int rand1 = rand() % cloudSize;
		int rand2 = rand() % cloudSize;
		while (rand2 == rand1) {
			rand2 = rand() % cloudSize;
		}
		int rand3 = rand() % cloudSize;
		while (rand3 == rand2 || rand3 == rand1) {
			rand3 = rand() % cloudSize;
		}

		PointT point1 = cloud->points[rand1];
		PointT point2 = cloud->points[rand2];
		PointT point3 = cloud->points[rand3];

		double x1 = point1.x;
		double x2 = point2.x;
		double x3 = point3.x;
		double y1 = point1.y;
		double y2 = point2.y;
		double y3 = point3.y;
		double z1 = point1.z;
		double z2 = point2.z;
		double z3 = point3.z;

		double i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		double j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		double k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

		double A = i;
		double B = j;
		double C = k;
		double D = -(i * x1 + j * y1 + k * z1);

		// Measure distance between every point and fitted line
		pcl::PointIndices::Ptr curInliers (new pcl::PointIndices());
		for (int index = 0; index < cloud->size(); index++) {
			PointT point = cloud->points[index];

			double x = point.x;
			double y = point.y;
			double z = point.z;
			double d = fabs(A * x + B * y + C * z + D) / sqrt(pow(A, 2) + pow(B, 2) + pow(C, 2));

			// If distance is smaller than threshold count it as inlier
			if (d < distanceTol) {
				curInliers->indices.push_back(index);
			}
		}
		if (curInliers->indices.size() > inliers->indices.size()) {
			*inliers = *curInliers;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliers;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(typename pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    RansacPlane(cloud, inliers, maxIterations, distanceThreshold);

    if (inliers->indices.size() == 0) {
        std::cerr << "Unable to estimate planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
bool ProcessPointClouds<PointT>::Proximity(const typename pcl::PointCloud<PointT>::Ptr cloud, int i, std::vector<int>& cluster, KdTree<PointT>* tree, float distanceTol, std::vector<bool>& processedPoints, int maxSize)
{
	processedPoints[i] = true;
	cluster.push_back(i);
    if (cluster.size() > maxSize)
    {
        return false;
    }
	std::vector<int> ids = tree->search(cloud->points[i], distanceTol);
	for (int id : ids)
	{
		if (!processedPoints[id])
        {
			bool continueClustering = Proximity(cloud, id, cluster, tree, distanceTol, processedPoints, maxSize);
            if (!continueClustering)
                // short-circuit the clustering function
                return false;
        }
	}
    return true;
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::EuclideanCluster(const typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int maxSize)
{
	std::vector<std::vector<int>> clusters;
	std::vector<bool> processedPoints(cloud->points.size(), false);

	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (processedPoints[i])
		{
			continue;
		}

		std::vector<int> cluster;
		bool keepCluster = Proximity(cloud, i, cluster, tree, distanceTol, processedPoints, maxSize);
        if (keepCluster)
        {
            clusters.push_back(cluster);
        }
	}

	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree<PointT>* myTree = new KdTree<PointT>();
    for (int i = 0; i < cloud->points.size(); i++)
    {
        myTree->insert(cloud->points[i], i);
    }

    std::vector<std::vector<int>> clustersIndices = EuclideanCluster(cloud, myTree, clusterTolerance, maxSize);

    for (std::vector<int> clusterIndices : clustersIndices)
    {
        if (clusterIndices.size() < minSize)
        {
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
        for (int index : clusterIndices)
        {
            cluster->points.push_back(cloud->points[index]);
        }

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}