#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <vector>
#include <numeric>
#include <math.h>

int Ktype = 0;
int MeanKtype = 0;
int removeType = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux1 (new pcl::PointCloud<pcl::PointXYZ>); //cloud after cutting

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

	// creates a new filtered point cloud
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
	sor.setMeanK (MeanKtype); //aumentar retira mais pontos
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

	float maxy = cloud->points[0].y;
	float valy = 0;

	for (size_t i = 0; i < cloud->points.size (); ++i){

		//max		
		if( maxy < cloud->points[i].y ){
			maxy =cloud->points[i].y;
			valy = i;
		}

	}

	//creating the highest point
	pcl::PointXYZ searchPoint (cloud->points[valy].x,cloud->points[valy].y,cloud->points[valy].z) ;
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
// Detects circular shapes
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr circularCloud3(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);	 

	std::cerr << " Initial cload with " << cloud->points.size() << "points " <<std::endl;

	std::vector<int> inliers;

	// created RandomSampleConsensus object and compute the appropriated model

	pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr    model_s (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
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
	
		// caso seja igual ao fim da lista, esse ponto nao pertence a lista de remover
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


////////////////////////////
// Decides scanning method and its parameteres
////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr ScanningType (int argc, char *argv[], const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aux2 (new pcl::PointCloud<pcl::PointXYZ>); //apenas aux

	if (pcl::console::find_switch (argc, argv, "-1")){

		std::cerr << "Big parallelepiped scanning" << std::endl;
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
	}

	if (pcl::console::find_switch (argc, argv, "-2")){

		std::cerr << "Big sphere scanning" << std::endl;
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

	}

	if (pcl::console::find_switch (argc, argv, "-3")){

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
	}

	

	if (pcl::console::find_switch (argc, argv, "-4")){

		std::cerr << "Cylinder scanning" << std::endl;
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

		//removes outliers
		MeanKtype = 100;
		cloud_final = FinalCleaning(cloud_final);
	}

	if (pcl::console::find_switch (argc, argv, "-5")){

		std::cerr << "Pyramid scanning" << std::endl;
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

		//removes outliers
		MeanKtype = 120; 
		cloud_final = FinalCleaning(cloud_final);
	}

	if (pcl::console::find_switch (argc, argv, "-6")){

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
		cloud_final = segmentCloud(cloud_aux1, centre);
		//apenas para o mais pequeno
		MeanKtype = 750;
		cloud_final = FinalCleaning(cloud_final);
		Ktype = 900;
		//measures the centre point on top 
		centre = meanPoint(cloud_final);
		//segments the cloud by the point
		cloud_final = segmentCloud(cloud_final, centre);
		
	}

	if (pcl::console::find_switch (argc, argv, "-7")){

		std::cerr << "Small sphere scanning" << std::endl;
		removeType = 130;
		//analysis area
		cloud_aux1 = cuttingCloud(cloud, -310, 310, -1020, -400);
		//obtains circular shapes
		cloud_final = circularCloud2(cloud_aux1);	
		cloud_final = FinalCleaning(cloud_final);
		cloud_final = borderCloud(cloud_final, 50);
		
	}

	
	return cloud_final;

}

int
main(int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final2 (new pcl::PointCloud<pcl::PointXYZ>);
	

	if (pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud1.pcd", *cloud) == -1){ //* load the file
	
		PCL_ERROR ("Couldn't read file cloud1.pcd \n");
		return (-1);
	}

	std::cerr << " Loaded cload with " << cloud->points.size() << "points " <<std::endl;

	cloud_final2 = ScanningType(argc, argv, cloud);
	
	pcl::io::savePCDFileASCII ("cloud2.pcd", *cloud_final2);

	pcl::visualization::PCLVisualizer viewer ("PCL visualizer");
	viewer.addPointCloud (cloud_aux1, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb (cloud_final2, 0, 255, 0);
	viewer.addPointCloud (cloud_final2, rgb ,"cloud outliers");

	while (!viewer.wasStopped ()) {
		viewer.spinOnce ();
	}

	return (0);
}

