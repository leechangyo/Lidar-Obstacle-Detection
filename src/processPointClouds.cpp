// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


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

// working real PCD project
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    //  filterres: defining the grid cell size of our vocel grid
    // "MinPoint, MaxPoint" are define the region in space(ROI)
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // STEP 1 :filtered cloud
    typename pcl::PointCloud<PointT>::Ptr filterd_cloud(new pcl::PointCloud<PointT>());
    // modeling filter(do voxel grid point)
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filterd_cloud); // saved filtered points in this cloud

    // STEP 2: SET ROI(we only going to be legt with points inside that box Region)
    typename pcl::PointCloud<PointT>::Ptr Region_cloud(new pcl::PointCloud<PointT>());

    pcl::CropBox<PointT> region(true); 
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filterd_cloud);
    region.filter(*Region_cloud); // save filtered points in regions cloud

    //Optional : fine the roof point indices and then feed thos indices to pcl extractindices object to remove them
    std::vector<int> indices; // points ID

    pcl::CropBox<PointT> roof(true); 
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(Region_cloud);
    roof.filter(indices); // now found out roof points by filtering, and save as points indices(we will outlier this points in the scene)
    std::cout<<"size of filtered indices : "<<indices.size()<<std::endl;

    // now we have filterd region cloud and roof points, we will apply roof points to region cloud to temove them(inlier work)
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int i : indices)
        inliers->indices.push_back(i);

    // Extract roof points ID in cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(Region_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*Region_cloud);

    // Optional end'

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return Region_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT> ()); // same type of pair on obstCloud
    typename pcl::PointCloud<PointT>::Ptr PlaneCloud(new pcl::PointCloud<PointT> ()); // same type of pair on obstCloud

    // planeCloud store from inliner
    for(int index : inliers->indices)
    {
        PlaneCloud->points.push_back(cloud->points[index]); // mapping index of inlier in cloud and store it at PlaneCloud
    }

    //Extract the inliers(generating obstacles
    pcl::ExtractIndices<PointT> extract;

    // store object
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, PlaneCloud);
    return segResult;
}
template<typename PointT>
void ProcessPointClouds<PointT>::CrossProduct(float v1[], float v2[], float cross_p[])
{
	cross_p[0] = v1[1]*v2[2]-v1[2]*v2[1];
	cross_p[1] = v1[2]*v2[0]-v1[0]*v2[2];
	cross_p[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

// RANSAC PLANE
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
   	std::unordered_set<int> inliersResult; // hold best inliers
	srand(time(NULL));

   		
    // TODO : project (3D envifonemnt)
    while(maxIterations--)
    {
    // step 1 : randomly pick two points(initlize. at begiing they don't have any inlier points that can not find out with coparing function)
        std::unordered_set<int> inliers;
        while(inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        // step 2: variable are given value by cloud(randomly picked)		
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        // pointing at the very beginning of inliers
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;	
        z2 = cloud->points[*itr].z;	
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;	
        z3 = cloud->points[*itr].z;		
        // step 3: use point 1 as a reference and define two vectors on the plane v1 and v2
        float v1[] = {x2-x1,y2-y1,z2-z1};
        float v2[] = {x3-x1,y3-y1,z3-z1};

        // find normal vector to plane by taking cross product of v1 x v2;
        float cross_p[3];
        CrossProduct(v1,v2,cross_p);

        // for plane easy calculation
        float A = cross_p[0];
        float B = cross_p[1];
        float C = cross_p[2];
        float D = -(A*x1+B*y1+C*z1);

        // step 4: distance between point and plane 
        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index)>0);
                continue; // if points already exist in our line, skip
            // if not calculate
            pcl::PointXYZ point = cloud->points[index];
            float x_ = point.x;
            float y_ = point.y;
            float z_ = point.z;

            // distance from the point to the plane is 
            float d_ = fabs(A*x_ + B*y_ + C*z_ +D )/sqrt(A*A + B*B + C*C);
            // threshold check and if converged condition, add in inliers
            if(d_<=distanceTol)
                inliers.insert(index);
        }

        if(inliers.size()>inliersResult.size())
            inliersResult = inliers; // when inlier bigger than previous result, then replace it with 
    }

    // seperate
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
    return segResult;
}

// why pair ? because they will segmanted into road and obstacle. and store
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();


    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices()); // stack method
    //pcl::PointIndices::Ptr* inliers = new pcl::PointIndices(); //heap method
    // pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // pcl::SACSegmentation<pcl::PointXYZ> seg; //create the segmentation object
    pcl::SACSegmentation<PointT> seg; //create the segmentation object (intensity)
    // Optional
    seg.setOptimizeCoefficients (true);
    // mandatory(Modeling, : ground detection)
    seg.setModelType(pcl::SACMODEL_PLANE); // determine object or plane
    seg.setMethodType(pcl::SAC_RANSAC); // use RANSAC algorithm find inlier data
    seg.setMaxIterations(maxIterations); //Ransac Iteration
    seg.setDistanceThreshold(distanceThreshold); //Ransac Threshold
    //Segment the Largest planar component from the remaining cloud

    seg.setInputCloud(cloud); // input data 
    seg.segment(*inliers, *coefficients); 
    // search inliers with coefficients for ground// outlier can be rest of them
    if(inliers->indices.size()==0)
    {
        std::cerr << "Could not estimate a plannar model for the given dataset." <<std::endl;
    }
    // END(seperated inlier)

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // STEP 1: creating the KD Tree objecto for the seach method of the extraction
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); // 3D
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);// includ intensity
    tree->setInputCloud(cloud); // set binary tree.

    // cluster resluts;
    std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; // how close they are by this funtion
    typename pcl::EuclideanClusterExtraction<PointT> ec; // include intenisty // clustering
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree); // clustering by euclidan cluster
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // clusterCloud(independantly) store from cluster_indices
    for(pcl::PointIndices getindices : cluster_indices)
    {
        // store object to cloud(different clusters) when use template need to use typename(automatically search type)
        typename pcl::PointCloud<PointT>::Ptr object_cl(new pcl::PointCloud<PointT> ()); // same type of pair on obstCloud
        
        // get each object with 
        for(int index : getindices.indices)
            object_cl->points.push_back(cloud->points[index]); // mapping index of inlier in cloud and store it at PlaneCloud

        object_cl->width = object_cl->points.size();
        object_cl->height = 1;
        object_cl->is_dense = true;

        clusters.push_back(object_cl); // push back to clusters with object cloud

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