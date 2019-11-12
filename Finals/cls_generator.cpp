#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <vector>

//tracks loaded original point clouds
int o_loaded = 0;

//tracks loaded generated point clouds
int n_loaded = 0;

//tracks saved point clouds
int n_saved = 0;

//angles to rotate in radians
std::vector<float> angles ;

//file names to read
std::vector<std::string> directories ;

////////////////////////////
// Measures distances
////////////////////////////
float squaredEuclideanDistance (const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){

	float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
	return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}

////////////////////////////
// Segments the cloud to the object to be scanned
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cuttingCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int xmin, const int xmax, const int ymin, const int ymax ){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition - >largura
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, xmin))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, xmax))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	// build the y condition -> comprimento
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condy (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, ymin))); //Greather than
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, ymax))); //Less than

	// build the y filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremy;
	condremy.setCondition (range_condy);
	condremy.setInputCloud (cloud_filtered);
	condremy.setKeepOrganized(true);
	condremy.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	std::cerr << "1 - Cloud before filtering: " << cloud->width << " " <<std::endl;
	std::cerr << "2 - Points filtered: " << num << " " <<std::endl;
	std::cerr << "3 - Cloud after filtering: " << cloud_filtered->width << " " <<std::endl;


	//creates a point cloud to save the new points filtered

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_filtered->width;
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i){
		if(!isnan(cloud_filtered->points[i].x)){
		cloud_final->points[num2].x = cloud_filtered->points[i].x;
		cloud_final->points[num2].y = cloud_filtered->points[i].y;
		cloud_final->points[num2].z = cloud_filtered->points[i].z;
		num2 ++;
		}		
	
	}

	return cloud_final;

}

////////////////////////////
// Removes the ground
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr groundRemoving(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZ>);

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane->values.resize (4);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (9);  // aumentar caso o ground seja ondulado diminiur caso seja mais simples

	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

	if (inliers_plane->indices.size () == 0) 
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");

	// Extract inliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane
	extract.setNegative (true);				// Extract the outliers
	  extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

	//criar a point cloud para guardar a nova e filtrada pcl

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width    = cloud_outliers->points.size ();
	cloud_final->height   = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize (cloud_final->width * cloud_final->height);

	int num2= 0;

	for (size_t i = 0; i < cloud_outliers->points.size (); ++i){
	
		cloud_final->points[num2].x = cloud_outliers->points[i].x;
		cloud_final->points[num2].y = cloud_outliers->points[i].y;
		cloud_final->points[num2].z = cloud_outliers->points[i].z;
		num2 ++;
			
	}

	std::cerr << "4 - Ground partially removed !" <<std::endl;

	return cloud_final;

}

////////////////////////////
// Removes the excedent points
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr pointsRemoving(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

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
	//std::cout << "Centre x- " << centre.x << " y- " << centre.y << " z- " << centre.z << ""<<std::endl;

	//saving all the distances to the centre
	std::vector<float> distances;
	float distance = 0;
	for (size_t i = 0; i < cloud->points.size (); ++i){
		

		distance = squaredEuclideanDistance(  cloud->points[i], centre);
		distances.push_back(distance);

	}

	//discovering the furthest points until the cloud as only 2048 points

	int removing = total_points - 2048;
	int detected = 0;

	std::vector<int> remove_points ;
	int farthest_value = distances[0];  //valor minimo e' o primeiro ponto conhecido
	int farthest_point = 0;

	int mean_distance1 = 0;
	int mean_distance = 0;

	for (size_t i = 0; i < distances.size (); ++i){

		mean_distance1 += distances[i];
	}

	mean_distance = mean_distance1 / distances.size();

	while(detected != removing){

		for (size_t i = 0; i < distances.size (); ++i){

			if(distances[i] > farthest_value){
				farthest_value = distances[i];
				farthest_point = i;
			}
		}

		//keep the index of the point to remove and remove other positions points 
		distances[farthest_point] = mean_distance; //with erase, the vector would substitute the deleted position by the deleted position +1
		detected++;
		remove_points.push_back(farthest_point);
		//std::cout << "point " << farthest_point << " removed !"<<std::endl;
		farthest_point = 0;
		farthest_value = distances[0];

	}

	std::cout << "5 - " << remove_points.size() << " points to be removed !"<<std::endl;

	// create new point cloud without the farthest points

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 2048;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);

	int num= 0; // auxiliar variable

	for (size_t i = 0; i < cloud->points.size (); ++i){
		
		// check if it is to remove
		std::vector<int>::iterator it = std::find(remove_points.begin(), remove_points.end(), i );
	
		// equal to the list end, so it isn't removed
		if(it == remove_points.end() && num <2048){ //avoid in some pcds to remove more than the necessary	
			cloud_final->points[num].x = cloud->points[i].x;
			cloud_final->points[num].y = cloud->points[i].y;
			cloud_final->points[num].z = cloud->points[i].z;
			num ++;
		}
	}

	return cloud_final;

}

////////////////////////////
// Rotates the cloud
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRotate(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float theta){

	// Using a Affine3f
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// The same rotation matrix, theta radians around Z axis, just change de UnitZ ou UnitX
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

	//std::cout << "Angle :" << theta * 180/M_PI << "º "<< std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud (*cloud, *transformed_cloud, transform_2);

	return transformed_cloud;

}

////////////////////////////
// Expand the points in the cloud
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDisperse(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int valuex, const int valuey, const int valuez){

	// Using a Affine3f
	Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();

	// Define a translation, float value, first x, second y, third z.
	transform_1.translation() << valuex, valuey, valuez;

	// Executing the transformations
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*cloud , *transformed_cloud1, transform_1);

	// Generating final cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);

	//Concatenating parts
	cloud_final = transformed_cloud1;

	return cloud_final;

}

////////////////////////////
// Saves point clouds
////////////////////////////
void cloudSave(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int save_number){

	//generating file name
	std::string file, end, result;
	file = "Generated/cloud";
	end = ".pcd";
	std::string number;
	std::stringstream ss;
	ss << save_number;
	number = ss.str();
	result =  file + number + end;

	// Saves point cloud
	pcl::io::savePCDFileASCII (result, *cloud);
	std::cout << "Point cloud saved in "<< result << "" <<std::endl;

	
}

////////////////////////////
// Loads original point clouds
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLoad(const int load_number, const std::string folder){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	//generating file name
	std::string file, end, result;
	file = "cloud";
	end = ".pcd";
	std::string number;
	std::stringstream ss;
	ss << load_number;
	number = ss.str();
	result = folder + file + number + end;
	
	// Loads point cloud
	if (pcl::io::loadPCDFile (result, *cloud) < 0) 
		PCL_ERROR ("Could not load PCD file !\n");

	std::cout << "Point cloud loaded from " << result << ""<<std::endl;
	return cloud;

}


////////////////////////////
// Generates different point clouds
////////////////////////////
// - Expands the points
// - Rotates each result of the expansion
////////////////////////////
void cloudGenerator(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	bool once = true;

	//generating more trainning samples
	for (size_t i = 5; i < 9; ++i){  //values of the expanding

		pcl::PointCloud<pcl::PointXYZ>::Ptr disperse_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		disperse_cloud = cloudDisperse(cloud, 0, i, 0);
		n_saved++;
		cloudSave(disperse_cloud, n_saved);

		disperse_cloud = cloudDisperse(cloud, i, 0, 0);
		n_saved++;
		cloudSave(disperse_cloud, n_saved);
		
		disperse_cloud = cloudDisperse(cloud, i, i, 0);
		n_saved++;
		cloudSave(disperse_cloud, n_saved);

	}
/*
	//loads the 13th generated pcd files with 0 degres
	for (size_t j = 0; j < 7; ++j){ // less 1 than the n_loaded

		pcl::PointCloud<pcl::PointXYZ>::Ptr rotate_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		//loading pcd
		n_loaded++;
		rotate_cloud = cloudLoad(n_loaded, "Generated/");

		//rotates each sample in 7 different degrees
		for (size_t k = 0; k < 7; ++k){  
			rotate_cloud = cloudRotate(rotate_cloud, angles[k]);
			n_saved++;
			cloudSave(rotate_cloud, n_saved);
		}
	}
*/
	//the next 0 degree clouds will be 13 numbers after
	n_loaded = n_loaded + 13;
	
}

////////////////////////////
// ciclo principal
////////////////////////////
int main (int argc, char** argv){

	//point cloud structure
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);

	//angles to rotate
	angles.push_back(M_PI/4);
	angles.push_back(M_PI/2);
	angles.push_back(3*M_PI/4);
	angles.push_back(M_PI);
	angles.push_back(5*M_PI/4);
	angles.push_back(3*M_PI/2);
	angles.push_back(7*M_PI/4);	

	//directories to load
	directories.push_back("Originals/Parallelepipeds/");
	directories.push_back("Originals/Spheres/");
	directories.push_back("Originals/Cubes/");
	directories.push_back("Originals/Cylinders/");
	directories.push_back("Originals/Pyramids/");
	
	//loads each file from the five different categories
	for (size_t i = 0; i < 5; ++i){ //directories

		for (size_t j = 0; j < 72; ++j){  //files

			//loading pcd
			o_loaded++;
			cloud = cloudLoad(o_loaded, directories[i]);

			//reduces the cloud to the desired size 2048 points
			cloud_final = cuttingCloud(cloud, -310, 310, -1020, -400);
			cloud_final = groundRemoving(cloud_final);
			cloud_final = pointsRemoving(cloud_final);

			std::cout << "6 - Final cloud with " << cloud_final->size() << " points !"<<std::endl;

			//saving pcd 
			n_saved++;
			cloudSave(cloud_final, n_saved);
			//generates 13 point clouds with same shape but different values
			cloudGenerator(cloud_final);

		}

		o_loaded = 0;

	}

	// point cloud extraction visualization

	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
	viewer.addPointCloud (cloud, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (cloud_final, 0, 255, 0);
	viewer.addPointCloud (cloud_final, rgb, "cloud outliers");

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}

	return (0);

}

