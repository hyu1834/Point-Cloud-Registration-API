# Point-Cloud-Registration-API
This module consist of point cloud registration methods discussed in "Methods for Calibration and Registration of Point Cloud Data Using Distance Geometry"

This module is implemented using PCL 1.7 as base point cloud data structure.

## Authors
Dr. Jian, Zhenxiang<br/>
University of California, Davis<br/>
Email: zxjian@ucdavis.edu <br/>
Linkedin: https://www.linkedin.com/in/zhenxiang-simon-jian-6101492b/
<br/>
<br/>
Yu, Hiu Hong<br/>
University of California, Davis<br/>
Email: hiuyu@ucdavis.edu<br/>
Linkedin: https://www.linkedin.com/in/hiuhong-yu-13220187/

## Background
The point cloud registration method in this tool is proposed by Dr. Jian, Zhenxiang, University of California, Davis and implemented by Yu, Hiu Hong and Dr. Jian, University of California, Davis.

The thesis can be found: http://search.proquest.com/docview/1832932748?pq-origsite=gscholar<br/>

## Registration Methods
- Point Cloud Registration Using Distance Geometry
- Bisecting Linear Line Complex Method for Point Cloud Registration

## Dependencies:
- PCL 1.7
- Eigen3

## To compile:
------
cd script && ./install_pcl1.7.sh

mkdir build

cd build

cmake ..

## To execute:
------
./bin/example/distance_geometry_registration

./bin/example/bisector_registration


