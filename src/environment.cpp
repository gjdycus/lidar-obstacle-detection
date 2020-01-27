/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc = lidar->scan();
    // renderRays(viewer, lidar->position, pc);
    // renderPointCloud(viewer, pc, "PointCloud");

    ProcessPointClouds<pcl::PointXYZ> pp;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> seg = pp.SegmentPlane(pc, 100, 0.2);
    // renderPointCloud(viewer, seg.first, "Obstacles", Color(1, 0, 0));
    // renderPointCloud(viewer, seg.second, "Plane", Color(0, 1, 0));
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pp.Clustering(seg.first, 1.0, 3, 30);
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters)
    {
        std::cout << "cluster size: ";
        pp.numPoints(cluster);
        Box box = pp.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % 3]);
        clusterId++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // downsample data and restrict to region of interest
    pointProcessorI->FilterCloud(inputCloud, 0.11, Eigen::Vector4f(-15.0, -5.0, -3.0, 1), Eigen::Vector4f(25.0, 7.0, 5.0, 1));

    // remove points from roof of ego car
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, Box> removalResult;
    removalResult = pointProcessorI->RemoveRegionFromCloud(inputCloud, Eigen::Vector4f(-1.5, -1.2, -1.0, 1), Eigen::Vector4f(2.6, 1.2, -0.4, 1));
    // renderBox(viewer, removalResult.second, 999);

    // separate ground from obstacles
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segments = pointProcessorI->SegmentPlane(inputCloud, 30, 0.2);

    // render ground
    renderPointCloud(viewer, segments.second, "planeCloud");

    // cluster obstacles
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering(segments.first, 0.43, 25, 2000);

    // loop through and render each cluster
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters)
    {
        std::cout << "cluster size: ";
        pointProcessorI->numPoints(cluster);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        renderPointCloud(viewer, cluster, "obstacleCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        clusterId++;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // clear view
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // load point cloud data
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

        // render current iteration
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // increment or reset iterator
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce (100);
    } 
}