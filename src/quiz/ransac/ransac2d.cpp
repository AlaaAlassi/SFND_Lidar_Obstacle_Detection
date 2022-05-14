/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
		pcl::PointXYZ p1;
		pcl::PointXYZ p2;
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
				float d = abs(A * point.x + B * point.y + C) / sqrt(A*A + B*B);
				// If distance is smaller than threshold count it as inlier
				if (d < distanceTol)
				{
					inliers.insert(idx);
				}
				idx++;
			}
		} else if (model == ModelType::plane)
		{
			while(inliers.size() < 3){
				inliers.insert(rand()%cloud.get()->size());
			}
			auto it = inliers.begin();
			auto p1 = cloud->points[*it];
			auto p2 = cloud->points[*it+1];
			auto p3 = cloud->points[*it+2];
			Eigen::Vector3d v1(p2.x - p1.x,		p2.y - p1.y,	p2.z-p1.z);
			Eigen::Vector3d v2(p3.x - p1.x,		p3.y - p1.y,	p3.z-p1.z);
			auto v3 = v1.cross(v2);

			double A = v3.x();
			double B = v3.y();
			double C = v3.z();
			double D = -(A*p1.x + B*p1.y + C*p1.z);

			for (int idx = 0;idx < cloud->size();idx++)
			{
				auto point = cloud->points[idx];
				if(inliers.count(idx)){
					continue;
				}

				float d = abs(A * point.x + B * point.y + C*point.z + D) / sqrt(A*A + B*B + C*C);
				// If distance is smaller than threshold count it as inlier
				if (d < distanceTol)
				{
					inliers.insert(idx);
				}
			}

			//cout << v3.x() <<" "<< v3.y() <<" "<< v3.z() << endl;
		}


		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}
	// Return indicies of inliers from fitted line with most inliers

	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 1000, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
