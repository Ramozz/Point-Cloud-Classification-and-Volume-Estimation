#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//volume
int formula = 0;
float comprimento = 0;
float largura = 0;
float altura = 0;
float AB = 0;
float diametro = 0;
pcl::PointXYZ the_point ;
float volume = 0;

//remove
int Ktype = 0;
int MeanKtype = 0;
int removeType = 0;

//visualization
bool mesh = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot (new pcl::PointCloud<pcl::PointXYZ>);
///////////////////////////////////////////////////////////////////////////////

////////////////////////////
// Segments the cloud to the object to be scanned
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cuttingCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int xmin, const int xmax, const int ymin, const int ymax ){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition 
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

	// build the y condition 
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


	//criar a point cloud para guardar a nova e filtrada pcl

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
//Segments the cloud 
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  segmentCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ centre){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	//creating octree
	float resolution = 128.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();

	// K nearest neighbor search

	int K = Ktype;

	std::vector<int> pointIdxNKNSearch;
	std::vector<float> pointNKNSquaredDistance;
	int num2=0;

	if (octree.nearestKSearch (centre, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){

		cloud_final->width = pointIdxNKNSearch.size();
		cloud_final->height = 1;
		cloud_final->is_dense = false;
		cloud_final->points.resize(cloud_final->width * cloud_final->height);

		for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i){
			cloud_final->points[num2].x = cloud->points[ pointIdxNKNSearch[i] ].x ;
			cloud_final->points[num2].y = cloud->points[ pointIdxNKNSearch[i] ].y ;
			cloud_final->points[num2].z = cloud->points[ pointIdxNKNSearch[i] ].z ;
			num2++;
		}

	}

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_final);
	sor.setMeanK (MeanKtype);
	sor.setStddevMulThresh (1.25);
	sor.filter (*cloud_filtered);

	//outliers
  	//sor.setNegative (true);
  	//sor.filter (*cloud_filtered);

	std::cerr << " Final cloud: " << cloud_filtered->points.size() << " points " <<std::endl;

	return cloud_filtered;
} 

////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  RandomSampleConsensus(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (15);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);

	return cloud_final;
} 

////////////////////////////
//final cleaning - statistical outlier removal
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  FinalCleaning(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_final);
	sor.setMeanK (MeanKtype);
	sor.setStddevMulThresh (1.25);
	sor.filter (*cloud_filtered);

	//outliers
  	//sor.setNegative (true);
  	//sor.filter (*cloud_filtered);

	std::cerr << " Final cloud: " << cloud_filtered->points.size() << " points " <<std::endl;

	return cloud_filtered;
}


// measuring the centre of the point cloud
pcl::PointXYZ meanPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
	
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

	return centre;
}

////////////////////////////
// discovers the highest point
////////////////////////////
pcl::PointXYZ highestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	float maxz = cloud->points[0].y;
	float valz = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){

		//max		
		if( maxz < cloud->points[i].z ){
			maxz =cloud->points[i].z;
			valz = i;
		}

	}

	//creating the highest point
	pcl::PointXYZ searchPoint (cloud->points[valz].x,cloud->points[valz].y,cloud->points[valz].z) ;
	return searchPoint;
}

////////////////////////////
// measures distance between two points
////////////////////////////
float squaredEuclideanDistance (const pcl::PointXYZ &p1, const pcl::PointXYZ &p2){

	float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
	return (diff_x*diff_x + diff_y*diff_y + diff_z*diff_z);
}

////////////////////////////
// Detects circular shapes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr circularCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr    model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
	ransac.setDistanceThreshold (15);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);


	return cloud_final;

}


////////////////////////////
// Detects circular shapes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr circularCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr    model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
	ransac.setDistanceThreshold (25);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);


	return cloud_final;

}

////////////////////////////
// Removes borders
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr borderCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const int points){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

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

	//saving all the distances to the centre
	std::vector<float> distances;
	float distance = 0;
	for (size_t i = 0; i < cloud->points.size (); ++i){
		

		distance = squaredEuclideanDistance( cloud->points[i], centre);
		distances.push_back(distance);

	}

	//discovering the farthest points until the cloud as only 2048 points

	int removing = points;
	int detected = 0;

	std::vector<int> remove_points ;
	int farthest_value = distances[0];  //min values is the first known point
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

		//guardo o indice do ponto a retirar e retiro a posicao das outras distancias 
		distances[farthest_point] = mean_distance; //with erase, the vector would substitute the deleted position by the deleted position +1
		detected++;
		remove_points.push_back(farthest_point);
		//std::cout << "point " << farthest_point << " removed !"<<std::endl;
		farthest_point = 0;
		farthest_value = distances[0];

	}

	// create new point cloud without the farthest points
	int newc = cloud->points.size() - remove_points.size();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = newc;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);

	int position = 0; // var auxiliar
	bool aux = false; // var auxiliar
	int num= 0; // var auxiliar
	std::vector<int>::iterator it;
	for (size_t i = 0; i < cloud->points.size (); ++i){

		for (size_t j = 0; j < remove_points.size (); ++j){

			if (i == remove_points[j]){
				aux = false;
				break;
			}
			
			else
				aux = true;
		}
	
		// if its equal to the end of the list, the point doesn't belong belong to the removing list
		if(aux && num <newc){	
			cloud_final->points[num].x = cloud->points[i].x;
			cloud_final->points[num].y = cloud->points[i].y;
			cloud_final->points[num].z = cloud->points[i].z;
			num ++;
			aux = false;
		}
	}

	std::cout << "Final cloud: " << cloud_final->points.size() << " points !"<<std::endl;

	return cloud_final;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7

//max and min

std::vector<float> maxmin(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	std::vector<float> values;

	float maxx = cloud->points[0].x;
	float Mvalx = 0;
	float maxy = cloud->points[0].y;
	float Mvaly = 0;
	float maxz = cloud->points[0].z;
	float Mvalz = 0;

	float minx = cloud->points[0].x;
	float mvalx = 0;
	float miny = cloud->points[0].y;
	float mvaly = 0;
	float minz = cloud->points[0].z;
	float mvalz = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){

		//max		
		if( maxx < cloud->points[i].x ){
			maxx =cloud->points[i].x;
			Mvalx = i;
		}

		if( maxy < cloud->points[i].y ){
			maxy =cloud->points[i].y;
			Mvaly = i;
		}

		if( maxz < cloud->points[i].z ){
			maxz =cloud->points[i].z;
			Mvalz = i;
		}

		//min
		if( minx > cloud->points[i].x ){
			minx =cloud->points[i].x;
			mvalx = i;
		}

		if( miny > cloud->points[i].y){
			miny =cloud->points[i].y;
			mvaly = i;
		}

		if( minz > cloud->points[i].z ){
			minz =cloud->points[i].z;
			mvalz = i;
		}

	}

	//lista com os valores max e minimos
	values.push_back(Mvalx);
	values.push_back(mvalx);
	values.push_back(Mvaly);
	values.push_back(mvaly);
	values.push_back(Mvalz);
	values.push_back(mvalz);

	return values;

}

////////////////////////////
// discovers the maximum and minimum points like puting an axis
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr valuePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	//distancia entre os max e min
	float centrex1 = cloud->points[maximus[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud->points[maximus[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud->points[maximus[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud->points[maximus[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	float centrez1 = cloud->points[maximus[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud->points[maximus[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	
	//min plus half the distance
	float pointx = cloud->points[maximus[1]].x + centrex/2;
	float pointy = cloud->points[maximus[3]].y + centrey/2;
	float pointz = cloud->points[maximus[5]].z + centrez/2;


	//scans by the central point
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			/*
			if(i == 4){
				cloud_final->points[i].x = 0;
				cloud_final->points[i].y = 0;
				cloud_final->points[i].z = 0;
			}
			*/	

			
		}
		else
			cloud_final->points[i] = the_point; //pontos central

		std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
	}

	return cloud_final;
}


////////////////////////////
// Segments the cloud to the object to be scanned -- based on height position z
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr zsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 70))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 70))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	// build the y condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condy (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, the_point.y ))); //Greather than
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, the_point.y + 10))); //Less than

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

	//new filtered point cloud
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

	std::vector<float> measures = maxmin(cloud_final);

	//distance between max and min
	
	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;
	
	comprimento = centrez/10; // to cm
	std::cerr << "comprimento result: " << comprimento << std::endl;

	return cloud_final;

}

//parall
////////////////////////////
// Segments the cloud to the object to be scanned
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr ysliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the z condition -> comprimento
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z ))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z + 10))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//new filtered point cloud
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

	std::vector<float> measures = maxmin(cloud_final);

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	
	altura = centrey/10; // to cm
	std::cerr << "altura result: " << altura << std::endl;
	return cloud_final;

}

//parall
////////////////////////////
// Segments the cloud to the object to be scanned - largura x
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr xsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the z condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z - 50))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z + 50))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	// build the x layer condition
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 300))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x ))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud_filtered);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//new filtered point cloud
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
	
	std::vector<float> measures = maxmin(cloud_final);

	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	largura = centrex/10 ; // to cm
	std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;

}

//cube
////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  PlaneRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (15);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);
	
	//altura media e valores abaixo dela
	
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

	// build the y condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condy (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, mean_y-55 ))); //Greather than
	range_condy->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, mean_y+55))); //Less than

	// build the y filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremy;
	condremy.setCondition (range_condy);
	condremy.setInputCloud (cloud_final);
	condremy.setKeepOrganized(true);
	condremy.filter (*cloud_final);

	std::vector<float> measures = maxmin(cloud_final);
	//distancia entre os max e min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	//because of the square detected can be different
	//if(centrez > centrex && centrez > centrey)
		//largura = centrez/10 ; // to cm

	if(centrey > centrex)
		largura = centrey/10 ; // to cm	

	else
		largura = centrex/10 ; // to cm
	

	std::cerr << "xresult: " << centrex << std::endl;
	std::cerr << "yresult: " << centrey << std::endl;
	//std::cerr << "zresult: " << centrez << std::endl;
	std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;
} 

//sphere
////////////////////////////
// discovers the maximum and minimum point
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr spherePoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	//distancia entre os max e min
	float centrex1 = cloud->points[maximus[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud->points[maximus[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrey1 = cloud->points[maximus[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud->points[maximus[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	float centrez1 = cloud->points[maximus[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud->points[maximus[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	
	//min plus half the distance
	float pointx = cloud->points[maximus[1]].x + centrex/2;
	float pointy = cloud->points[maximus[3]].y + centrey/2;
	float pointz = cloud->points[maximus[5]].z + centrez/2;


	//scans by the central point
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			

		}
		else
			cloud_final->points[i] = the_point; //central points
		std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
		
	}

	return cloud_final;
}

//sphere
////////////////////////////
// Segments the cloud to the object to be scanned - largura x
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the z condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z - 50))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z + 50))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 10))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 300))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud_filtered);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//new filtered point cloud
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
	
	std::vector<float> measures = maxmin(cloud_final);

	//distance between min and max
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	largura = centrex/10 ; // to cm
	std::cerr << "largura result: " << largura << std::endl;

	return cloud_final;

}


//cyl
////////////////////////////
// discovers the highest point
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cylPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	cloud_final->width = 7;
	cloud_final->height = 1;
	cloud_final->is_dense = false;
	cloud_final->points.resize(cloud_final->width * cloud_final->height);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	float centrey1 = cloud->points[maximus[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud->points[maximus[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	
	//min plus half the distance
	float pointx = cloud->points[maximus[2]].x;
	float pointy = cloud->points[maximus[3]].y + centrey/2;
	float pointz = cloud->points[maximus[2]].z ;


	//scans by the central point
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			

		}
		else
			cloud_final->points[i] = the_point; //central points
			//std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
		
	}

	return cloud_final;
}

//cyl
////////////////////////////
// Segments the cloud to the object to be scanned
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr cylsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
/*
	// build the z condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z  -10))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z + 10))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);
*/
	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 25))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 25))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);

	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//new filtered point cloud
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

	std::vector<float> measures = maxmin(cloud_final);

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;

	//distance between max and min
	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	diametro = centrex/10; //to cm
	altura = centrey/10; // to cm
	std::cerr << "altura result: " << altura <<  "x result: "<< diametro <<std::endl;
	return cloud_final;

}

//pyr
////////////////////////////
//Removes planes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr  groundRemove(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr  model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (10);
	ransac.computeModel();
	ransac.getInliers(inliers);

	// copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *cloud_final);

	//obtaining outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointIndices::Ptr the_inliers (new pcl::PointIndices());
	the_inliers->indices = inliers;

	extract.setInputCloud(cloud);
	extract.setIndices(the_inliers);
	extract.setNegative(true);
	extract.filter(*cloud_final);
	std::vector<float> maximus;

	maximus = maxmin(cloud);

	float centrey1 = cloud_final->points[maximus[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[maximus[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;
	
	
	//minimo mais metade da distancia
	float pointx = cloud->points[maximus[2]].x;
	float pointy = cloud->points[maximus[3]].y + centrey/2;
	float pointz = cloud->points[maximus[2]].z ;


	//ponto de scan e o ponto mais alto
	pcl::PointXYZ the_point1 (pointx,pointy,pointz) ;
	the_point = the_point1;
	std:cerr << the_point << std::endl;

	for(size_t i =0; i< cloud_final->points.size(); ++i){

		if(i<6){
			cloud_final->points[i].x = cloud->points[maximus[i]].x;
			cloud_final->points[i].y = cloud->points[maximus[i]].y;
			cloud_final->points[i].z = cloud->points[maximus[i]].z;
			

		}
		else
			cloud_final->points[i] = the_point; //central points
			//std::cerr <<  ""<< i << " x- "<< cloud->points[maximus[i]].x << " y- "<< cloud->points[maximus[i]].y << " z- "<< cloud->points[maximus[i]].z <<  std::endl;
		
	}

	return cloud_final;
} 


//pyr
////////////////////////////
// cuts two slices to the pyr
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr pyrsliceCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// build the z condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condz (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, the_point.z - 50))); //Greather than
	range_condz->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, the_point.z + 50))); //Less than

	// build the z filter and apply
	pcl::ConditionalRemoval<pcl::PointXYZ> condremz;
	condremz.setCondition (range_condz);
	condremz.setInputCloud (cloud);
	condremz.setKeepOrganized(true);
	condremz.filter (*cloud_filtered);

	// build the x layer condition 
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condx (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, the_point.x - 80))); //Greather than
	range_condx->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
	pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, the_point.x + 90))); //Less than

	// build the x filter and apply 
	pcl::ConditionalRemoval<pcl::PointXYZ> condremx;
	condremx.setCondition (range_condx);
	condremx.setInputCloud (cloud_filtered);
	condremx.setKeepOrganized(true);
	condremx.filter (*cloud_filtered);


	//number of filtered points
	int num = 0;

	for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
		if(isnan(cloud_filtered->points[i].x)){
			num++;
			cloud_filtered->width--;
		}

	//criar a point cloud para guardar a nova e filtrada pcl

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

	std::vector<float> measures = maxmin(cloud_final);

	float centrey1 = cloud_final->points[measures[2]].y; 
	centrey1 = centrey1 * centrey1;
	centrey1 = sqrt(centrey1);
	float centrey2 = cloud_final->points[measures[3]].y; 
	centrey2 = centrey2 * centrey2;
	centrey2 = sqrt(centrey2);
	float centrey = 0;
	if(centrey1 > centrey2)
		centrey = centrey1 - centrey2;
	else
		centrey = centrey2 - centrey1;

	float centrex1 = cloud_final->points[measures[0]].x; 
	centrex1 = centrex1 * centrex1;
	centrex1 = sqrt(centrex1);
	float centrex2 = cloud_final->points[measures[1]].x; 
	centrex2 = centrex2 * centrex2;
	centrex2 = sqrt(centrex2);
	float centrex = 0;
	if(centrex1 > centrex2)
		centrex = centrex1 - centrex2;
	else
		centrex = centrex2 - centrex1;

	float centrez1 = cloud_final->points[measures[4]].z; 
	centrez1 = centrez1 * centrez1;
	centrez1 = sqrt(centrez1);
	float centrez2 = cloud_final->points[measures[5]].z; 
	centrez2 = centrez2 * centrez2;
	centrez2 = sqrt(centrez2);
	float centrez = 0;
	if(centrez1 > centrez2)
		centrez = centrez1 - centrez2;
	else
		centrez = centrez2 - centrez1;

	//usar o valor mais pequeno
	if(centrez > centrex)
		comprimento = centrex/10; //to cm
	else
		comprimento = centrez/10;
	
	altura = centrey/10; // to cm
	std::cerr << "altura : " << altura << " comprimento: " << comprimento <<  std::endl;
	return cloud_final;

}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////
// Generates point cloud triangle mesh
////////////////////////////
pcl::PolygonMesh meshingCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (10);   // posso trocar o search radius aumentar ou diminuir

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	return triangles;
}

////////////////////////////
// Measures triangle volume
////////////////////////////
float SignedVolumeOfTriangle(const pcl::PointXYZ p1,const  pcl::PointXYZ p2, const pcl::PointXYZ p3) {
    float v321 = p3.x*p2.y*p1.z;
    float v231 = p2.x*p3.y*p1.z;
    float v312 = p3.x*p1.y*p2.z;
    float v132 = p1.x*p3.y*p2.z;
    float v213 = p2.x*p1.y*p3.z;
    float v123 = p1.x*p2.y*p3.z;
    return (1.0f/6000.0f)*(-v321 + v231 + v312 - v132 - v213 + v123); // dividin for 1000 because it's cm^3
}

////////////////////////////
// Generates volume of point cloud
////////////////////////////
float VolumeOfMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PolygonMesh mesh) {

	std::vector<float> vols ;
	float volume = 0;
	float final_volume = 0;
	int ix = 0;
	int iy = 0;
	int iz = 0;

	for (size_t i = 0 ; i< mesh.polygons.size(); ++i){
		ix = mesh.polygons[i].vertices[0];
		iy = mesh.polygons[i].vertices[1];
		iz = mesh.polygons[i].vertices[2];
		
		volume =  SignedVolumeOfTriangle(cloud->points[ix], cloud->points[iy], cloud->points[iz]);
		vols.push_back(volume);

	}

	//sums all the triangles
	final_volume = accumulate(vols.begin(),vols.end(),0);
	final_volume = sqrt (final_volume * final_volume);
	return final_volume;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////
// Decides scanning method and its parameteres
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr SlicesVolume (int argc, char *argv[], const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final1 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux1 (new pcl::PointCloud<pcl::PointXYZ>); //permite alteracoes na cloud original
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux2 (new pcl::PointCloud<pcl::PointXYZ>); //apenas aux
  
	if (pcl::console::find_switch (argc, argv, "-11")){

		std::cerr << "Big parallelepiped scanning ..." << std::endl;
		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);
		cloud_plot = cloud_final1;
		std::cerr << "Parallelepiped volume ..." << std::endl;
		//descovers where is the object
		cloud_final2 = valuePoint(cloud_final1);
		//extract characteristics
		cloud_final2 = zsliceCloud(cloud_final1);
		cloud_aux = xsliceCloud(cloud_final1);
		cloud_aux = ysliceCloud(cloud_final1);
		
		volume = comprimento * altura * largura;
		comprimento = 0;
		altura = 0;
		largura = 0;
	}

	if (pcl::console::find_switch (argc, argv, "-12")){

		std::cerr << "Big sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_aux1 = circularCloud(cloud_aux1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_aux1);
		//removes points in the borders
		cloud_final1 = borderCloud(cloud_final1, removeType);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		cloud_plot = cloud_final1;		
		std::cerr << "Big sphere volume ..." << std::endl;
		//detects sphere points
		cloud_aux = spherePoint(cloud_final1);
		//obtains diameter
		cloud_final2 = sphereCloud(cloud_final1);
		diametro = largura;
		float raio = diametro /2;
		float r3 = pow (raio, 3.0);
		volume = (4*M_PI* r3) /3 ;
	
	}

	if (pcl::console::find_switch (argc, argv, "-13")){

		std::cerr << "Cube scanning" << std::endl;
		//grande
		//Ktype = 1000;
		//MeanKtype = 650;
		Ktype = 700;
		MeanKtype = 450;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);
		cloud_plot = cloud_final1;
		std::cerr << "Cube volume ..." << std::endl;
		cloud_final2 = PlaneRemove (cloud_final1);
		float c3 = pow(largura, 3.0);
		volume = c3;
		largura = 0;

	}

	if (pcl::console::find_switch (argc, argv, "-14")){

		std::cerr << "Cylinder scanning" << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_final1);

		//removes outliers
		MeanKtype = 100;
		cloud_final1 = FinalCleaning(cloud_final1);
		cloud_plot = cloud_final1;
		std::cerr << "Cylinder volume ..." << std::endl;
		cloud_final2 = cylPoint(cloud_final1);
		cloud_final2 = cylsliceCloud(cloud_final1);
		float raio = diametro /2;
		float r2 = pow (raio, 2.0);
		volume = M_PI*r2* altura;
		

	}

	if (pcl::console::find_switch (argc, argv, "-15")){

		std::cerr << "Pyramid scanning" << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final1 = RandomSampleConsensus(cloud_final1);
		//removes outliers
		cloud_final1 = FinalCleaning(cloud_final1);
		cloud_plot = cloud_final1;
		std::cerr << "Pyramid volume ... " << std::endl;
		cloud_final2 = groundRemove (cloud_final1);
		cloud_final2 = pyrsliceCloud(cloud_final1);
		
		AB = comprimento * comprimento;
		volume = (AB * altura) / 3;
	}

	if (pcl::console::find_switch (argc, argv, "-16")){

		std::cerr << "Small parallelepiped scanning" << std::endl;
		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_aux1, centre);
		//only for the smallest
		MeanKtype = 750;
		cloud_final1 = FinalCleaning(cloud_final1);
		Ktype = 900;
		//measures the centre point on top 
		centre = meanPoint(cloud_final1);
		//segments the cloud by the point
		cloud_final1 = segmentCloud(cloud_final1, centre);

		std::cerr << "Small parallelepiped volume ..." << std::endl;
		//discovers where is the object
		cloud_final2 = valuePoint(cloud_final1);
		//discovers characteristics
		cloud_final2 = zsliceCloud(cloud_final1);
		cloud_aux = xsliceCloud(cloud_final1);
		cloud_aux = ysliceCloud(cloud_final1);
		cloud_plot = cloud_final1;
		volume = comprimento * altura * largura;
		comprimento = 0;
		altura = 0;
		largura = 0;
	}

	if (pcl::console::find_switch (argc, argv, "-17")){

		std::cerr << "Small sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final1 = circularCloud2(cloud_aux1);
		
		cloud_final1 = FinalCleaning(cloud_final1);
		cloud_final1 = borderCloud(cloud_final1, 50);
		cloud_plot = cloud_final1;

		std::cerr << "Small sphere volume ..." << std::endl;
		//detecta esfera
		cloud_aux = spherePoint(cloud_final1);
		//obtem diametro
		cloud_final2 = sphereCloud(cloud_final1);
		diametro = largura;
		float raio = diametro /2;
		float r3 = pow (raio, 3.0);
		volume = (4*M_PI* r3) /3 ;
	}

	//output cloud
	return cloud_final2;
}

////////////////////////////
// Decides mesh scanning method and its parameteres
////////////////////////////
pcl::PolygonMesh MeshVolume (int argc, char *argv[], const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux1 (new pcl::PointCloud<pcl::PointXYZ>); //permite alteracoes na cloud original
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux2 (new pcl::PointCloud<pcl::PointXYZ>); //apenas aux
	pcl::PolygonMesh mesh_final ;
	
	float factor =0;

	if (pcl::console::find_switch (argc, argv, "-21")){

		std::cerr << "Parallelepiped scanning ..." << std::endl;

		Ktype = 2500;
		MeanKtype = 1500;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final = segmentCloud(cloud_aux1, centre);
		//apenas para o mais pequeno
		/*
		MeanKtype = 700;
		cloud_final = FinalCleaning(cloud_final);
		Ktype = 850;
		//measures the centre point on top 
		centre = meanPoint(cloud_final);
		//segments the cloud by the point
		cloud_final = segmentCloud(cloud_final, centre);
		*/

		std::cerr << "Parallelepiped volume" << std::endl;
		factor =19.73;
		mesh = true;

	}

	if (pcl::console::find_switch (argc, argv, "-22")){

		std::cerr << "Sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_aux1 = circularCloud(cloud_aux1);
		//removes outliers
		cloud_final = FinalCleaning(cloud_aux1);
		//removes points in the borders
		cloud_final = borderCloud(cloud_final, removeType);
		//removes the rest of the ground
		cloud_final = RandomSampleConsensus(cloud_final);
		//pequeno
	/*
		//removes points in the borders
		cloud_final = borderCloud(cloud_final, 250);
		//removes outliers
		MeanKtype = 50;
		cloud_final = FinalCleaning(cloud_final);
		cloud_final = borderCloud(cloud_final, 20);
		*/

		std::cerr << "Sphere volume" << std::endl;
		factor =25.23;
		mesh = true;
	
	}

	if (pcl::console::find_switch (argc, argv, "-23")){

		std::cerr << "Cube scanning" << std::endl;
		//grande
		//Ktype = 1000;
		//MeanKtype = 650;
		Ktype = 700;
		MeanKtype = 450;
		pcl::PointXYZ centre;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//removes ground
		cloud_aux1 = RandomSampleConsensus(cloud_aux1);
		//obtains the top plane
		cloud_aux2 = RandomSampleConsensus(cloud_aux1);
		//measures the centre point on top 
		centre = meanPoint(cloud_aux2);
		//segments the cloud by the point
		cloud_final = segmentCloud(cloud_aux1, centre);
		//apenas para o mais pequeno
		/*
		MeanKtype = 700;
		cloud_final = FinalCleaning(cloud_final);
		Ktype = 850;
		//measures the centre point on top 
		centre = meanPoint(cloud_final);
		//segments the cloud by the point
		cloud_final = segmentCloud(cloud_final, centre);
		*/

		std::cerr << "Cube volume" << std::endl;
		factor = 6.45 ;
		mesh = true;

	}

	if (pcl::console::find_switch (argc, argv, "-24")){

		std::cerr << "Pyramid scanning ..." << std::endl;
		removeType = 600;
		Ktype = 1100;
		MeanKtype = 700;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final = circularCloud(cloud_aux1);
		//removes outliers
		cloud_final = FinalCleaning(cloud_final);
		//removes points in the borders
		cloud_final = borderCloud(cloud_final, removeType);
		//fino/*
		//measures the centre point on top 
		pcl::PointXYZ centre;
		centre = meanPoint(cloud_final);
		//segments the cloud by the point
		//Ktype = 600; para o mais pequeno
		cloud_final = segmentCloud(cloud_final, centre);


		std::cerr << "Pyramid volume" << std::endl;
		factor = 1.97 ;
		mesh = true;

	}

	if (pcl::console::find_switch (argc, argv, "-25")){

		std::cerr << "Cylinder scanning ..." << std::endl;
		removeType = 130;
		Ktype = 300;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final = circularCloud(cloud_aux1);
		//removes the rest of the ground
		cloud_final = RandomSampleConsensus(cloud_final);
		//removes outliers
		cloud_final = FinalCleaning(cloud_final);
/*
		pcl::PointXYZ centre;
		centre = highestPoint(cloud_final);
		//segments the cloud by the point
		Ktype = 110;
		cloud_final = segmentCloud(cloud_final, centre);
		
		*/

		std::cerr << "Cylinder volume" << std::endl;
		factor = 9.54 ;
		mesh = true;

	}

	if(mesh){
		mesh_final = meshingCloud(cloud);
		std::cerr << "Polygons: " << mesh_final.polygons.size() << " " <<std::endl;
		volume = VolumeOfMesh(cloud, mesh_final);
		volume = volume * factor;
	}

	return mesh_final;
}

int
main (int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh mesh_final ;

	// Load point cloud
	if (pcl::io::loadPCDFile ("cloud1.pcd", *cloud) < 0) {
		PCL_ERROR ("Could not load PCD file !\n");
		return (-1);
	}
	
	cloud_final = SlicesVolume (argc, argv, cloud);

	mesh_final = MeshVolume(argc, argv, cloud);

	std::cerr << "Volume result: " << volume << std::endl;

	if(mesh){

		//mesh visualization
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->addPolygonMesh(mesh_final,"meshes",0);

		while (!viewer->wasStopped ()){
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
	}
	else{
		//normal visualization
		pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
		viewer.addPointCloud (cloud_plot, "cloud");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (cloud_final, 255 ,0, 0);
		viewer.addPointCloud (cloud_final, rgb, "cloud outliers");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud outliers");

		while (!viewer.wasStopped ()) {
			viewer.spinOnce ();
		}
	}

	
	return (0);
}
