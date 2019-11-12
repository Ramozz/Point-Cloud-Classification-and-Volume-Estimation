#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud1.pcd", *cloud) == -1) //* load the file
	{
	PCL_ERROR ("Couldn't read file cloud1.pcd \n");
	return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from cloud.pcd." << std::endl;

	// measuring the centre of the point cloud
	float total_x = 0;
	float total_y = 0;
	float total_z = 0;
	float mean_x = 0;
	float mean_y = 0;
	float mean_z = 0;
	float total_points = cloud->points.size ();

	for (size_t i = 0; i < cloud->points.size (); ++i){
			
		total_x += cloud->points[i].x;
		total_y += cloud->points[i].y;
		total_z += cloud->points[i].z;		
	}

	mean_x = total_x / total_points ;
	mean_y = total_y / total_points ;
	mean_z = total_z / total_points ;

	//creating the centre point
	pcl::PointXYZ centre (mean_x,mean_y,mean_z) ;

	///////////////////////////////
	//creating the transformations
	///////////////////////////////

	//parametros de entrada
	int value1 = 8; //largura
	int value2 = 8; // altura alterar sempre
	int value3 = 8; //nunca mexer em z

	// Using a Affine3f
	Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();

	// Define a translation, float value, first x, second y, third z.
	transform_1.translation() << -value1, -value2, -value3;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.translation() << -value1, value2, -value3;

	Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
	transform_3.translation() << -value1, value2, value3;

	Eigen::Affine3f transform_4 = Eigen::Affine3f::Identity();
	transform_4.translation() << -value1, -value2, value3;

	Eigen::Affine3f transform_5 = Eigen::Affine3f::Identity();
	transform_5.translation() << value1, -value2, -value3;

	Eigen::Affine3f transform_6 = Eigen::Affine3f::Identity();
	transform_6.translation() << value1, value2, -value3;

	Eigen::Affine3f transform_7 = Eigen::Affine3f::Identity();
	transform_7.translation() << value1, value2, value3;

	Eigen::Affine3f transform_8 = Eigen::Affine3f::Identity();
	transform_8.translation() << value1, -value2, value3;

	///////////////////////////////
	//creating 8 point clouds
	///////////////////////////////

	int num1=0, num2 = 0, num3 =0, num4 = 0, num5=0, num6= 0, num7 = 0, num8 = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){
		if(cloud->points[i].x < centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z < centre.z )
			num1++;

		if(cloud->points[i].x < centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z < centre.z )
			num2++;

		if(cloud->points[i].x < centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z >= centre.z )
			num3++;

		if(cloud->points[i].x < centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z >= centre.z )
			num4++;

		if(cloud->points[i].x >= centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z < centre.z )
			num5++;

		if(cloud->points[i].x >= centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z < centre.z )
			num6++;

		if(cloud->points[i].x >= centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z >= centre.z )
			num7++;

		if(cloud->points[i].x >= centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z >= centre.z )
			num8++;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr part1 (new pcl::PointCloud<pcl::PointXYZ>);
	part1->width = num1;
	part1->height = 1;
	part1->is_dense = false;
	part1->points.resize(part1->width * part1->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part2 (new pcl::PointCloud<pcl::PointXYZ>);
	part2->width = num2;
	part2->height = 1;
	part2->is_dense = false;
	part2->points.resize(part2->width * part2->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part3 (new pcl::PointCloud<pcl::PointXYZ>);
	part3->width = num3;
	part3->height = 1;
	part3->is_dense = false;
	part3->points.resize(part3->width * part3->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part4 (new pcl::PointCloud<pcl::PointXYZ>);
	part4->width = num4;
	part4->height = 1;
	part4->is_dense = false;
	part4->points.resize(part4->width * part4->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part5 (new pcl::PointCloud<pcl::PointXYZ>);
	part5->width = num5;
	part5->height = 1;
	part5->is_dense = false;
	part5->points.resize(part5->width * part5->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part6 (new pcl::PointCloud<pcl::PointXYZ>);
	part6->width = num6;
	part6->height = 1;
	part6->is_dense = false;
	part6->points.resize(part6->width * part6->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part7 (new pcl::PointCloud<pcl::PointXYZ>);
	part7->width = num7;
	part7->height = 1;
	part7->is_dense = false;
	part7->points.resize(part7->width * part7->height);
	pcl::PointCloud<pcl::PointXYZ>::Ptr part8 (new pcl::PointCloud<pcl::PointXYZ>);
	part8->width = num8;
	part8->height = 1;
	part8->is_dense = false;
	part8->points.resize(part8->width * part8->height);

	num1=0, num2=0, num3=0, num4=0, num5=0, num6=0, num7=0, num8=0;

	for (size_t i = 0; i < cloud->points.size (); ++i){
		if(cloud->points[i].x < centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z < centre.z ){
			part1->points[num1] = cloud->points[i];
			num1++;
		}

		if(cloud->points[i].x < centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z < centre.z ){
			part2->points[num2] = cloud->points[i];
			num2++;
		}

		if(cloud->points[i].x < centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z >= centre.z ){
			part3->points[num3] = cloud->points[i];
			num3++;
		}

		if(cloud->points[i].x < centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z >= centre.z ){
			part4->points[num4] = cloud->points[i];
			num4++;
		}

		if(cloud->points[i].x >= centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z < centre.z ){
			part5->points[num5] = cloud->points[i];
			num5++;
		}

		if(cloud->points[i].x >= centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z < centre.z ){
			part6->points[num6] = cloud->points[i];
			num6++;
		}

		if(cloud->points[i].x >= centre.x && cloud->points[i].y >= centre.y  && cloud->points[i].z >= centre.z ){
			part7->points[num7] = cloud->points[i];
			num7++;
		}

		if(cloud->points[i].x >= centre.x && cloud->points[i].y < centre.y  && cloud->points[i].z >= centre.z ){
			part8->points[num8] = cloud->points[i];
			num8++;
		}

	}

	// Executing the transformations
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*cloud , *transformed_cloud1, transform_1);


	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part2 , *transformed_cloud2, transform_2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud3 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part3 , *transformed_cloud3, transform_3);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud4 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part4 , *transformed_cloud4, transform_4);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud5 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part5 , *transformed_cloud5, transform_5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud6 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part6 , *transformed_cloud6, transform_6);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud7 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part7 , *transformed_cloud7, transform_7);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud8 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*part8 , *transformed_cloud8, transform_8);

	///////////////////////////////
	// Generating final cloud 
	///////////////////////////////

	//Concatenating parts
	*cloud_final += *transformed_cloud2;
	*cloud_final += *transformed_cloud3;
	*cloud_final += *transformed_cloud4;
	*cloud_final += *transformed_cloud5;
	*cloud_final += *transformed_cloud6;
	*cloud_final += *transformed_cloud7;
	*cloud_final += *transformed_cloud8;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);

	cloud_final = transformed_cloud1;
	///////////////////////////////
	// Visualization
	///////////////////////////////

	pcl::visualization::PCLVisualizer viewer ("Point Cloud Rotation");
	// Define R,G,B colors for the point cloud
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
	viewer.addPointCloud (cloud, cloud_color_handler, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_final_color_handler (cloud_final, 0, 255, 0); // Green
	viewer.addPointCloud (cloud_final, cloud_final_color_handler, "cloud_final");
	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_final");

	pcl::io::savePCDFileASCII ("cloud_final.pcd", *cloud_final);
	std::cerr << "Saved " << cloud_final->points.size () <<" points to cloud_final.pcd." << std::endl;


	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed

		viewer.spinOnce ();
	}


	return (0);
}

