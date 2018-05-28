// Standard Libraries
#include <iostream>

// 3rd Parties Libraries
//PCL
#include <pcl/common/common_headers.h>

//Eigen
#include <Eigen/Dense>

//Local Libraries
#include <point_cloud_utils.h>
#include <distance_geometry_registration.h>


int main(int argc, char** argv)	{
	if(argc < 4)	{
		std::cerr << "Usage: distance_geometry_registration <SourceCorrespondence> <SourcePointCloud> <TargetCorrespondence>\n";
		return -1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCorrespondence(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr targetCorrespondence(new pcl::PointCloud<pcl::PointXYZRGB>);


	// Import Point Cloud
	importPointCloud<pcl::PointXYZRGB>(argv[1], sourceCorrespondence);
	importPointCloud<pcl::PointXYZRGB>(argv[2], sourcePointCloud);
	importPointCloud<pcl::PointXYZRGB>(argv[3], targetCorrespondence);

	DistanceGeometryRegistration<pcl::PointXYZRGB>* dgr = new DistanceGeometryRegistration<pcl::PointXYZRGB>();
	dgr->pointCloudRegistration(sourceCorrespondence, targetCorrespondence, sourcePointCloud, THREAD_MODE::THREAD_DISABLE);
	std::cout << "Transformation Error: " << dgr->estimateError(sourceCorrespondence, targetCorrespondence);

	exportPointCloud<pcl::PointXYZRGB>("transformatedPointCloud.txt", sourcePointCloud);

	delete dgr;
	return 0;
}