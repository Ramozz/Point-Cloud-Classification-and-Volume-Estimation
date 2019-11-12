# Point-Cloud-Classification-and-Volume

This project is divided in two parts: the Python part, classifies point clouds with PointNet architecture and the C++ part, extracts and measures the volume of the point clouds with PCL and OpenCV.

This repository is composed by the code developed along my dissertation. The theme was " Polygon Classification and Volume estimation based in TOF point clouds", where theclassification task is performed by a Deep Learning architecture in Python and the remaining procedures used for measuring the polygon's volume were developed in C++. 

The point cloud gathering was performed by Hitachi-LG's TOF sensor HLS L-FOM5.

In this work, 6 main tasks are performed:
- Dataset augmentation;
- Points removal and filtering;
- Point cloud classification;
- Mesh generation;
- Volume estimation using Slices;
- Volume estimation using tetrahedrons.

## Libraries

To performe those tasks, the main libraries used were: 
- PCL;
- OpenCV;
- H5PY;
- PyntCloud;
- Numpy;
- OS;
- Keras;

## Structure

There are 2 main sections: 
The individuals, which are scripts developed separatelly for later being included in the main module, some of them were not used in the final code, but may provide useful ideas and examples, containing:
- cloud_remove - contains the functions for extracting the polygon's points from the surronding environment
- cloud_volume - initial prototype for extracting the polygon and volume estimation
- cloud_viewer - reads a pcd file a displays on the screen
- cloud_2048 - cuts to fit the analysis square and removes the less important points until only remain 2048 points
- cloud_20482 - second approach to reduce the point cloud to only 2048 points 
- cloud_voxel - example how to create a voxel grid
- cloud_disperse - moves the points away from its centre in the 3 axis directions

The finals, which correspond to all the modules used in the dissertation execution.
- cls_keras_train - which is responsible to train the PointNet architecture, based in https://github.com/garyli1019/pointnet-keras and to generated a h5 file with the resulting weights after the training.
- cls_keras_predict - contains a TCP server and the Deep Learning architecture, when it receives a message, it will read the point cloud and predict its class.
- cls_preparation - reads the pcd files, stores them in Numpy arrays and generated h5 files which will be later used for the network train.
- human_counter - contains the required code for the TOF sensor along with the methods developed to clean, measure and estimate the Polygon's volume.
- cls_generator - takes the point clouds captured, cuts, transpose and rotates them in order to generate more point clouds and create a bigger dataset, which could provide better classification results.

Multiple datasets were created in order to enhance the accuracy of the classification network and the polygons used were the following:

![1](https://user-images.githubusercontent.com/39749315/68707811-623cd100-058a-11ea-9a65-dd881f4c4f69.JPG)

The classification_weights.h5 is also available consisting in node weights, which correspond to the best performance achieved during the PointNet train.

## Results:

### Classification:

![2](https://user-images.githubusercontent.com/39749315/68708096-e8591780-058a-11ea-9c12-206a10f5b8ba.JPG)

The worst values obtained in the classification were due to the color of the small Cube, whose dark characteristics difficult the reflection of the light beam and consequently, a bad point cloud reconstruction. The other reason is the proximity to the ground along the whole prototype development, which resulted in a difficulty to separete and identify small objects from the ground.

It was detected that the network is also capable of correctly classify objects with different sizes of the ones used for training.


### Volume:

- #### Mesh:

![4](https://user-images.githubusercontent.com/39749315/68708102-eabb7180-058a-11ea-8124-abedd25c9a40.JPG)

- #### Slices:

![5](https://user-images.githubusercontent.com/39749315/68708106-eb540800-058a-11ea-8d87-b8c2a541b9a8.JPG)

The Mesh algorithm generates a volume prediction in 6 seconds and the Slices algorithm processes only in 2 seconds.

## Final considerations

This repository is also useful for PCL begginers with implementation examples of:
- Statistical Outlier Removal;
- K- nearest neighour;
- Octree;
- Random Sample Consensus for spheres and planes;
- Voxel Grid;
- Translations;
- Visualizer;
- SacSegmentation;
- Conditional Removal.

Which can serve as useful examples for teaching how to use other functions of the open source library.

A CMake file is also available for the possible need of generating an executable.

