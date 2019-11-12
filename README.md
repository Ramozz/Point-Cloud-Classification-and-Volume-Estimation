# Point-Cloud-Classification-and-Volume
This project is divided in two parts: the python part, classifies point clouds with PointNet architecture and the C++ part, extracts and measures the volume of the point clouds with PCL and OpenCV.

This repository is composed by the code developed along my dissertation. The theme was " Polygon Classification and Volume estimation based in TOF point clouds", where theclassification task is performed by a Deep Learning architecture in Python and the remaining procedures used for measuring the polygon's volume were developed in C++. The point cloud gathering was performed by Hitachi-LG's TOF sensor HLS L-FOM5.

In this work, 6 main tasks are performed:
- Dataset augmentation;
- Points removal and filtering;
- Point cloud classification;
- Mesh generation;
- Volume estimation using Slices;
- Volume estimation using tetrahedrons.

To performe those tasks, the main libraries used were: 
- PCL;
- OpenCV;
- H5PY;
- Numpy

This repository is also useful for PCL begginers with implementation examples of:
- Statistical Outlier Removal
- K- nearest neighour
- Octree
- Random Sample Consensus for spheres and planes
- Voxel Grid
which can serve as useful examples for teaching how to use other functions of the open source library.
