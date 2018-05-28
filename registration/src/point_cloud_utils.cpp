#include "point_cloud_utils.h"

/*
	Point Cloud Methods
*/
Eigen::MatrixXf calculateTranslationVector(Eigen::MatrixXf rotationMatrix, pcl::PointXYZ centroid1, 
											  pcl::PointXYZ centroid2)	{
	Eigen::MatrixXf translationVector;
		
	// first convert the centroid into Eigen 3x1 column vector
	Eigen::MatrixXf cen1(3, 1);
	Eigen::MatrixXf cen2(3, 1);

	cen1 << centroid1.x, centroid1.y, centroid1.z;
	cen2 << centroid2.x, centroid2.y, centroid2.z;

	translationVector = -rotationMatrix * cen1 + cen2;

	return translationVector;
}
