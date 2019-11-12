#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>

int
main (int argc, char** argv){
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	//constructs the string to be loaded
	char* choise = argv[1];
	std::string file = "cloud";
	std::string end = ".pcd";
	std::string result;
	result = file + choise + end;

	//* loads the file
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (result, *cloud) == -1){ 
	
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	
	std::cout << "Loaded "
			<< cloud->width * cloud->height
			<< " data points from cloud.pcd with the following fields: "
			<< std::endl;

	int num = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){
		if(cloud->points[i].x != 0 && cloud->points[i].y != 0){
				num++;/*
				std::cout << "    " << cloud->points[i].x
					  << " "    << cloud->points[i].y
					  << " "    << cloud->points[i].z << std::endl;*/
			}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	cloud2->width = num;
	cloud2->height = 1;
	cloud2->is_dense = false;
	cloud2->points.resize(cloud2->width * cloud2->height);
	std::cout << "  choise " << choise << std::endl;
	std::cout << "    " << cloud2->width
				  << " "    << cloud2->height << std::endl;
	int num2= 0;

	//removes points with all coordinates == 0
	for (size_t i = 0; i < cloud->points.size (); ++i){
		if(cloud->points[i].x != 0 && cloud->points[i].y != 0){
					cloud2->points[num2].x = cloud->points[i].x;
				cloud2->points[num2].y = cloud->points[i].y;
				cloud2->points[num2].z = cloud->points[i].z;
				num2 ++;		
			}
	}

	//save the point cloud
	pcl::io::savePCDFileASCII ("cloud2.pcd", *cloud2);
	std::cerr << "Saved " << cloud2->points.size () <<" points to cloud2.pcd." << std::endl;
	std::cerr << "First " << cloud2->points[0].x <<" points to cloud2.pcd." << cloud2->points[1].y<<std::endl;


	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

	viewer.addPointCloud (cloud, "cloud outliers");	

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}

	return (0);
}

