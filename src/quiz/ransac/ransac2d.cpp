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
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

void CrossProduct(float v1[], float v2[], float cross_p[])
{
	cross_p[0] = v1[1]*v2[2]-v1[2]*v2[1];
	cross_p[1] = v1[2]*v2[0]-v1[0]*v2[2];
	cross_p[2] = v1[0]*v2[1]-v1[1]*v2[0];
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult; // hold best inliers
	srand(time(NULL));
	
	// TODO: Fill in this function(quiz)

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	/*
	while(maxIterations--)
	{
		// step 1 : randomly pick two points(initlize. at begiing they don't have any inlier points that can not find out with coparing function)
		std::unordered_set<int> inliers;
		while(inliers.size()<2)
			// rand can be very large number and by doing this modulo, then just returing some value between zero and the points size from cloud.
			// if element is existed in inliers because of data type of unorered_set, it doens't duplicate(count) it.
			// so means this reandomly pick cloud, and if same value encounter, ignore.
			inliers.insert(rand()%(cloud->points.size()));// if less than 2, it going to randomly insert point.
		// step 2: variable are given value by cloud(randomly picked)		
		float x1, y1, x2, y2;
		// pointing at the very beginning of inliers
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		// line formular
		float A = (y1-y2);
		float B = (x2-x1);
		float C = (x1*y2 - x2*y1);

		// step 3: iterate throught the all points of data 
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// we just seeing how if it contains an element of not
			// if not zero, then it does contain the element, so we are not going to actually do anything with it.
			// because it is already pary of our line;
			if(inliers.count(index)>0);
				continue;
			
			// if not we will check points whether contain or not with calculation

			pcl::PointXYZ point = cloud->points[index]; // takt point
			float x3 = point.x;
			float y3 = point.y;

			// fabs : absolute functions(float), find distance using below equations("Best line search algorithm")
			float d = fabs(A*x3 + B*y3 + C) / sqrt(A*A + B*B);
			if(d<=distanceTol)
				inliers.insert(index);
			

		}
		if(inliers.size()>inliersResult.size())
			inliersResult = inliers; // when inlier bigger than previous result, then replace it with 

		*/
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
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();//2D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
