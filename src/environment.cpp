/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);

    auto ptCloud = lidar->scan();

    // renderRays(viewer,lidar->position,ptCloud);

    // renderPointCloud(viewer,ptCloud,"PointCloud#1",Color(1,1,1));

    ProcessPointClouds<pcl::PointXYZ> processPCD;

    // if this flag is false, the function will skip PCL segmentation and use my implementation
    bool usingPCL = false;
    auto result = processPCD.SegmentPlane(ptCloud, 100, 0.2,usingPCL);
    float clusterTolerance = 1;
    int minSize = 4;
    int maxSize = 30;
    auto detectedClusters = processPCD.Clustering(result.second, clusterTolerance, minSize, maxSize);

    // renderPointCloud(viewer,result.first,"planeCloud#2",Color(1,0,0));
    // renderPointCloud(viewer,result.second,"obstaclesCloud#3",Color(0,1,0));
    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 1, 1), Color(0, 0, 1)};

    int i = 0;
    for (auto const &cluster : detectedClusters)
    {
        Box box = processPCD.BoundingBox(cluster);
        renderBox(viewer, box, i);
        renderPointCloud(viewer, cluster, "obstacle#" + std::to_string(i), colors[i]);
        i++;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{


    // filter cloud
    auto filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, {-20, -6, -2, 1}, {20, 6, 0, 1});
    // renderPointCloud(viewer,filteredCloud,"inputCloud");

    // segment cloud
    ProcessPointClouds<pcl::PointXYZI> processPCD;
    // if this flag is false, the function will skip PCL segmentation and use my implementation
    bool usingPCL = false;
    auto segmentedCloud = processPCD.SegmentPlane(filteredCloud, 100, 0.2,usingPCL);
    renderPointCloud(viewer, segmentedCloud.first, "street", Color(0, 1, 0));
    // renderPointCloud(viewer,segmentedCloud.second,"obsticle",Color(1,1,1));

    // cluster the obsticle cloud
    float clusterTolerance = 0.3;
    int minSize = 20;
    int maxSize = 2000;
    //auto detectedClusters = processPCD.Clustering(segmentedCloud.second, clusterTolerance, minSize, maxSize);
    auto detectedClusters = processPCD.eucledianClustering(segmentedCloud.second, clusterTolerance, minSize, maxSize);


    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 0), Color(0, 0, 1)};

    int i = 0;
    for (auto const &cluster : detectedClusters)
    {
        Box box = processPCD.BoundingBox(cluster);
         renderBox(viewer,box,i);
        renderPointCloud(viewer, cluster, "obstacle#" + std::to_string(i), colors[i % 3]);
        i++;
    }
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    auto stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        auto inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce();
    }
}