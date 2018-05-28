// Standard Libraries
#include <iostream>

// 3rd Parties Libraries
//PCL
#include <pcl/common/common_headers.h>

//Eigen
#include <Eigen/Dense>

//Local Libraries
#include <point_cloud_utils.h>
#include <bisecting_linear_line_complex_registration.h>

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

	BisectingLinearLineComplexRegistration<pcl::PointXYZRGB>* bllcr = new BisectingLinearLineComplexRegistration<pcl::PointXYZRGB>();
	if(!bllcr->computeTransformation(sourceCorrespondence, targetCorrespondence))	{
		std::cerr << "Error: Unable to compute transformation\n";
		return -1;
	}

	if(!bllcr->pointCloudTransformation(sourcePointCloud))	{
		std::cerr << "Error: Unable to transform source point cloud\n";
		return -1;
	}

	exportPointCloud<pcl::PointXYZRGB>("transformatedPointCloud.txt", sourcePointCloud);

	delete bllcr;
	return 0;
}