// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // remove roof detected points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto idx : indices)
    {
        inliers->indices.push_back(idx);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>());

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);

    extract.setNegative(true);
    extract.filter(*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(planeCloud, obstaclesCloud);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                                                 int maxIterations,
                                                                                                                                 float distanceThreshold,
                                                                                                                                 bool usindPCL)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // create PointIndices pointer
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    if (usindPCL)
    {
        // create segmentation object
        pcl::SACSegmentation<PointT> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);

        // set model coefficient
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
    }
    else
    {
        auto result = Ransac(cloud, maxIterations, distanceThreshold);
        for (auto it = result.begin(); it != result.end(); it++)
        {
            inliers->indices.push_back(*it);
        }
    }

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    pcl::EuclideanClusterExtraction<PointT> ec;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx : it->indices)
        {

            cloud_cluster->push_back((*cloud)[idx]); //*
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const vector<vector<float>> &points,
                                           std::vector<int> &cluster,
                                           vector<bool> &processed,
                                           int idx,
                                           KdTree *tree,
                                           float distanceTol)
{
    processed[idx] = true;
    cluster.push_back(idx);
    auto nearest = tree->search(points[idx], distanceTol);
    for (auto const id : nearest)
    {
        if (!processed[id])
        {
            proximity(points, cluster, processed, id, tree, distanceTol);
        }
    }
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::eucledianClustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                   float clusterTolerance,
                                                                                                   int minSize,
                                                                                                   int maxSize)
{

    KdTree *tree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> v{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        tree->insert(v, i);
        points.push_back(v);
    }

    std::vector<bool> processed(points.size(), false);
    std::vector<std::vector<int>> clusters;
    for (int i = 0; i < points.size(); i++)
    {
        if (!processed[i])
        {
            std::vector<int> cluster;
            proximity(points, cluster, processed, i, tree, clusterTolerance);
            clusters.push_back(cluster);
        }
    }

    // fill cluster clouds in a vector
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;

    for (std::vector<int> cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for (int indice : cluster)
            clusterCloud->points.push_back(cloud->points[indice]);
        clusterClouds.push_back(clusterCloud);
    }

    return clusterClouds;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    enum ModelType
    {
        line,
        plane
    };
    ModelType model = ModelType::plane;
    // TODO: Fill in this function

    // For max iterations
    for (int i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> inliers;

        if (model == ModelType::line)
        {
            // Randomly sample subset and fit line
            int p1RandomIdx = 0;
            int p2RandomIdx = 0;
            PointT p1;
            PointT p2;
            do
            {
                p1RandomIdx = rand() % cloud.get()->size();
                p1 = cloud.get()->points[p1RandomIdx];
                p2RandomIdx = rand() % cloud.get()->size();
                ;
                p2 = cloud.get()->points[p2RandomIdx];
            } while (p1RandomIdx == p2RandomIdx);
            // Line Coefficients
            double A = (p1.y - p2.y);
            double B = (p2.x - p1.x);
            double C = (p1.x * p2.y) - (p2.x * p1.y);

            // Measure distance between every point and fitted line
            int idx = 0;
            for (auto const &point : cloud.get()->points)
            {
                float d = abs(A * point.x + B * point.y + C) / sqrt(A * A + B * B);
                // If distance is smaller than threshold count it as inlier
                if (d < distanceTol)
                {
                    inliers.insert(idx);
                }
                idx++;
            }
        }
        else if (model == ModelType::plane)
        {
            while (inliers.size() < 3)
            {
                inliers.insert(rand() % cloud.get()->size());
            }
            auto it = inliers.begin();
            auto p1 = cloud->points[*it];
            it++;
            auto p2 = cloud->points[*it];
            it++;
            auto p3 = cloud->points[*it];
            Eigen::Vector3d v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
            Eigen::Vector3d v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
            auto v3 = v1.cross(v2);

            double A = v3.x();
            double B = v3.y();
            double C = v3.z();
            double D = -(A * p1.x + B * p1.y + C * p1.z);

            for (int idx = 0; idx < cloud->size(); idx++)
            {
                auto point = cloud->points[idx];
                if (inliers.count(idx))
                {
                    continue;
                }

                float d = abs(A * point.x + B * point.y + C * point.z + D) / sqrt(A * A + B * B + C * C);
                // If distance is smaller than threshold count it as inlier
                if (d < distanceTol)
                {
                    inliers.insert(idx);
                }
            }

            // cout << v3.x() <<" "<< v3.y() <<" "<< v3.z() << endl;
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}