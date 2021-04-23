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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // TO DO : create Processor
    // ProcessPointClouds<pcl::PointXYZI>* pointProcessor(new ProcessPointClouds<pcl::PointXYZI>());
    // TODO : Load Real PCD 
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // ToDO : filter cloud(ROI and voxel: to reduce data set)
    Eigen::Vector4f minpoint (-31,-5.5,-2,1); // minimum staring region
    Eigen::Vector4f maxpoint (31,5.5,9,1); // maximum ending region
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, 0.2, minpoint, maxpoint);
    std::cout<<"filtersize:"<<filterCloud->width *filterCloud->height << endl;

    // TODO : Steps for Obstacle Detection
    // 1. Segment the filtered cloud into two parts, road and obstacles
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud,99,0.3); // cloud, itetartion, therhold distance(for ransac)
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); // plane

    // // 2. Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudclusters = pointProcessor->Clustering(segmentCloud.first, 0.5,9, 4900); // obstacle cloud, clustertolerance, mincluster, max cluster

    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudclusters)
    {
        std::cout << "cluster size";
        pointProcessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"ObjectCloud"+std::to_string(clusterID),colors[clusterID%colors.size()]);
        
        // Bounding box
        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterID);
        ++clusterID;
    }   


}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false; // just get render itself. without car
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    // make lidar to touch cars
    Lidar* lidar = new Lidar(cars,0); //creating lidar object


    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(lidar->scan()); // creating pointcloud, call the lidar sacn() method on our lidar object and store pointcloud pointer object
    // renderRays(viewer, lidar->position, inputCloud); // render our pointcloud as line segmentes in the viewer
    // renderPointCloud(viewer,inputCloud, "inputCloud");// at this time we dont set postion, because render rays knew that  how the rays starting the postion.
    // name associated with because this time we can have multiple pointcloud showing up in the viewer and each one identified.

    // Method 1 : Segment plane and object
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // generate processPointCLouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud,100,0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // Method 2: Ransac Plane
    /*
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.RansacPlane(inputCloud,100,0.2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    */

    // cluster method
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudclusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudclusters)
    {
        std::cout << "cluster size";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"ObjectCloud"+std::to_string(clusterID),colors[clusterID%colors.size()]);
        
        // Bounding box
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterID);
        ++clusterID;
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

    // simulate
    // simpleHighway(viewer);

    // real PCD file
    // cityBlock(viewer);

    // stream PCD 
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor(new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_2/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr InputCloud;

    while (!viewer->wasStopped ())
    {
        //clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //load pcd and run obstacle detection process
        InputCloud = pointProcessor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, InputCloud);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce ();
    } 
}