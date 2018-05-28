#ifndef PCL_POINT_CLOUD_H
	#define PCL_POINT_CLOUD_H

/*
	Standard Libraries
*/
#include <iostream>
#include <climits>
#include <utility>
#include <fstream>
#include <string>
#include <cassert>
#include <vector>
#include <typeinfo>
#include <map>

/*
	3rd Parties Libraries
*/
//PCL
#include <pcl/common/common_headers.h>
#include <pcl/ModelCoefficients.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

/*
	Local Libraries
*/

/*
	Enum
*/
enum AXIS	{
	X_AXIS = 0,
	Y_AXIS,
	Z_AXIS
};

/*
	MACRO
*/
#define DEBUGGING
#define POINTXY_ID typeid(pcl::PointXY).name()
#define POINTXYZ_ID typeid(pcl::PointXYZ).name()
#define POINTXYZRGB_ID typeid(pcl::PointXYZRGB).name()


/*
	Point Cloud Methods
*/
Eigen::MatrixXf calculateTranslationVector(Eigen::MatrixXf rotationMatrix, pcl::PointXYZ centroid1, 
										   pcl::PointXYZ centroid2);

/*
	Template Point Methods
*/
template<typename PointT>
void importPointCloud(std::string filename, typename pcl::PointCloud<PointT>::Ptr pc, 
					  double xOffset = 0.0, double yOffset = 0.0, double zOffset = 0.0)	{
	std::ifstream in(filename, std::ifstream::in);
	double x, y, z;

	if(pc == NULL)	{
		return ;
	}
	// clear all existing point cloud
	pc->clear();

	// check if its PointXYZ or PointXYZRGB
	if(typeid(PointT).name() == POINTXYZ_ID)	{
		while(in >> x >> y >> z)	{
			pc->push_back(PointT());
			pc->back().x = x + xOffset;
			pc->back().y = y + yOffset;
			pc->back().z = z + zOffset;
		}
	}
	else if(typeid(PointT).name() == POINTXYZRGB_ID)	{
		unsigned int r, g, b;
		while(in >> x >> y >> z >> r >> g >> b)	{
			pc->push_back(PointT(r, g, b));
			pc->back().x = x + xOffset;
			pc->back().y = y + yOffset;
			pc->back().z = z + zOffset;
		}
	}

	in.close();
}

template<typename PointT>
void importPointCloud(std::istream& in, typename pcl::PointCloud<PointT>::Ptr pc, 
					  double xOffset = 0.0, double yOffset = 0.0, double zOffset = 0.0)	{
	double x, y, z;
	if(pc == NULL)	{
		return ;
	}
	// clear all existing point cloud
	pc->clear();
	

	// check if its PointXYZ or PointXYZRGB
	if(typeid(PointT).name() == POINTXYZ_ID)	{
		while(in >> x >> y >> z)	{
			pc->push_back(PointT());
			pc->back().x = x + xOffset;
			pc->back().y = y + yOffset;
			pc->back().z = z + zOffset;
		}
	}
	else if(typeid(PointT).name() == POINTXYZRGB_ID)	{
		unsigned int r, g, b;
		while(in >> x >> y >> z >> r >> g >> b)	{
			pc->push_back(PointT(r, g, b));
			pc->back().x = x + xOffset;
			pc->back().y = y + yOffset;
			pc->back().z = z + zOffset;
		}
	}
}

template<typename PointT2D, typename PointT3D>
void importPointCloud(std::string filename, typename pcl::PointCloud<PointT2D>::Ptr pc2D, typename pcl::PointCloud<PointT3D>::Ptr pc3D, 
					  double xOffset = 0.0, double yOffset = 0.0, double zOffset = 0.0)	{
	double x, y, z;
	// File descriptor
	std::ifstream in(filename, std::ifstream::in);

	// Clear both point cloud container
	if(pc2D == NULL || pc3D == NULL)	{
		return ;
	}
	pc2D->clear();
	pc3D->clear();

	// Check the type of container
	if((typeid(PointT2D).name() == POINTXY_ID) && (typeid(PointT3D).name() == POINTXYZ_ID))	{
		while(in >> x >> y >> z)	{
			pc2D->push_back(PointT2D());
			pc2D->back().x = x + xOffset;
			pc2D->back().y = y + yOffset;

			pc3D->push_back(PointT3D());
			pc3D->back().x = x + xOffset;
			pc3D->back().y = y + yOffset;
			pc3D->back().z = z + zOffset;
		}
	}
	else if((typeid(PointT2D).name() == POINTXY_ID) && (typeid(PointT3D).name() == POINTXYZRGB_ID))	{
		unsigned int r, g ,b;
		while(in >> x >> y >> z >> r >> g >> b)	{
			pc2D->push_back(PointT2D());
			pc2D->back().x = x + xOffset;
			pc2D->back().y = y + yOffset;

			pc3D->push_back(PointT3D(r, g, b));
			pc3D->back().x = x + xOffset;
			pc3D->back().y = y + yOffset;
			pc3D->back().z = z + zOffset;
		}
	}
	else if((typeid(PointT2D).name() == POINTXYZ_ID) && (typeid(PointT3D).name() == POINTXYZ_ID))	{
		while(in >> x >> y >> z)	{
			pc2D->push_back(PointT2D());
			pc2D->back().x = x + xOffset;
			pc2D->back().y = y + yOffset;

			pc3D->push_back(PointT3D());
			pc3D->back().x = x + xOffset;
			pc3D->back().y = y + yOffset;
			pc3D->back().z = z + zOffset;
		}
	}
	else if((typeid(PointT2D).name() == POINTXYZ_ID) && (typeid(PointT3D).name() == POINTXYZRGB_ID))	{
		unsigned int r, g ,b;
		while(in >> x >> y >> z >> r >> g >> b)	{
			pc2D->push_back(PointT2D());
			pc2D->back().x = x + xOffset;
			pc2D->back().y = y + yOffset;

			pc3D->push_back(PointT3D(r, g, b));
			pc3D->back().x = x + xOffset;
			pc3D->back().y = y + yOffset;
			pc3D->back().z = z + zOffset;
		}
	}

	in.close();
}

template<typename PointT>
void exportPointCloud(std::string filename, typename pcl::PointCloud<PointT>::Ptr pc, double xOffset = 0.0, double yOffset = 0.0, double zOffset = 0.0)	{
	// File Descriptor
	std::ofstream out(filename, std::ifstream::out);

	// check if its PointXYZ or PointXYZRGB
	if(typeid(pcl::PointXYZ) == typeid(PointT))	{
		for(typename pcl::PointCloud<PointT>::iterator it = pc->begin(); it != pc->end(); it++)	{
			out << std::setprecision(10) << it->x - xOffset << "  " << it->y - yOffset << "  " << it->z - zOffset << "\n";
		}
	}
	else if(typeid(pcl::PointXYZRGB) == typeid(PointT))	{
		for(typename pcl::PointCloud<PointT>::iterator it = pc->begin(); it != pc->end(); it++)	{
			out << std::setprecision(10) << it->x - xOffset << "  " << it->y - yOffset << "  " << it->z - zOffset << "  " <<
											(unsigned int)it->r << "  " << (unsigned int)it->g << "  " << (unsigned int)it->b << "\n";
		}
	}
}

template<typename PointT>
void exportPointCloud(std::ofstream& out, typename pcl::PointCloud<PointT>::Ptr pc, double xOffset = 0.0, double yOffset = 0.0, double zOffset = 0.0)	{
	// check if its PointXYZ or PointXYZRGB
	if(typeid(pcl::PointXYZ) == typeid(PointT))	{
		for(typename pcl::PointCloud<PointT>::iterator it = pc->begin(); it != pc->end(); it++)	{
			out << std::setprecision(10) << it->x - xOffset << "  " << it->y - yOffset << "  " << it->z - zOffset << "\n";
		}
	}
	else if(typeid(pcl::PointXYZRGB) == typeid(PointT))	{
		for(typename pcl::PointCloud<PointT>::iterator it = pc->begin(); it != pc->end(); it++)	{
			out << std::setprecision(10) << it->x - xOffset << "  " << it->y - yOffset << "  " << it->z - zOffset << "  " <<
											(unsigned int)it->r << "  " << (unsigned int)it->g << "  " << (unsigned int)it->b << "\n";
		}
	}
}


template<typename PointT>
PointT rotation3D(PointT point, int degree, AXIS axis)	{
	double radian = degree * (M_PI / 180.0);

	if(axis == X_AXIS)	{
		point.y = point.y * cos(radian) - point.z * sin(radian);
		point.z = point.y * sin(radian) + point.z * cos(radian);
	}
	else if(axis == Y_AXIS)	{
		point.z = point.z * cos(radian) - point.x * sin(radian);
		point.x = point.z * sin(radian) + point.x * cos(radian);
	}
	else {
		point.x = point.x * cos(radian) - point.y * sin(radian);
		point.y = point.x * sin(radian) + point.y * cos(radian);
	}	

	return point;
}

/*
	Template Point Cloud Methods
*/
template <typename PointT>
pcl::PointXYZ calculatePCLCentroid(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
	pcl::PointXYZ centroid(0.0, 0.0, 0.0);

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		centroid.x += it->x;
		centroid.y += it->y;
		centroid.z += it->z;
	}

	centroid.x /= pointCloud->size();
	centroid.y /= pointCloud->size();
	centroid.z /= pointCloud->size();

	return centroid;
}

template <typename PointT>
Eigen::Vector3d calculateEigenCentroid(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
	Eigen::Vector3d centroid(0.0, 0.0, 0.0);

	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		centroid(0) += it->x;
		centroid(1) += it->y;
		centroid(2) += it->z;
	}

	centroid(0) /= pointCloud->size();
	centroid(1) /= pointCloud->size();
	centroid(2) /= pointCloud->size();

	return centroid;
}

template <typename PointT>
void reCenterPointCloud(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
	pcl::PointXYZ centroid = calculatePCLCentroid<PointT>(pointCloud);

	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		it->x -= centroid.x;
		it->y -= centroid.y;
		it->z -= centroid.z;
	}
}

template <typename PointT>
Eigen::MatrixXd computeCovarianceMatrix(typename pcl::PointCloud<PointT>::Ptr pointCloud1, typename pcl::PointCloud<PointT>::Ptr pointCloud2)	{
	Eigen::Vector3d centroid1 = calculateEigenCentroid<PointT>(pointCloud1);
	Eigen::Vector3d centroid2 = calculateEigenCentroid<PointT>(pointCloud2);

	//Covariance Matrix container, is a 3x3 matrix
	Eigen::MatrixXd covarianceMatrix(3, 3);
	//set all to 0
	covarianceMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	// Make sure all size are =
	if(pointCloud1->size() != pointCloud2->size())	{
		return covarianceMatrix;
	}
	// calculate covariance matrix
	for(int i = 0; i < pointCloud1->size(); i++)	{
		//calculate sub matrix
		Eigen::Vector3d temp1, temp2;
		//construct the 1x3 and 3x1 matrix
		temp1 << pointCloud1->points[i].x, pointCloud1->points[i].y, pointCloud1->points[i].z;
		temp2 << pointCloud2->points[i].x, pointCloud2->points[i].y, pointCloud2->points[i].z;
		temp1 -= centroid1;
		temp2 -= centroid2;

		covarianceMatrix += (temp1 * temp2.transpose());
	}

	return covarianceMatrix;
}

template <typename PointT>
Eigen::MatrixXd computeCovarianceMatrix(typename pcl::PointCloud<PointT>::Ptr pointCloud1, Eigen::Vector3d centroid1, 
										typename pcl::PointCloud<PointT>::Ptr pointCloud2, Eigen::Vector3d centroid2)	{
	//Covariance Matrix container, is a 3x3 matrix
	Eigen::MatrixXd covarianceMatrix(3, 3);
	//set all to 0
	covarianceMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	// Make sure all size are =
	if(pointCloud1->size() != pointCloud2->size())	{
		return covarianceMatrix;
	}
	// calculate covariance matrix
	for(int i = 0; i < pointCloud1->size(); i++)	{
		//calculate sub matrix
		Eigen::Vector3d temp1, temp2;
		//construct the 1x3 and 3x1 matrix
		temp1 << pointCloud1->points[i].x, pointCloud1->points[i].y, pointCloud1->points[i].z;
		temp2 << pointCloud2->points[i].x, pointCloud2->points[i].y, pointCloud2->points[i].z;
		temp1 -= centroid1;
		temp2 -= centroid2;

		covarianceMatrix += (temp1 * temp2.transpose());
	}

	return covarianceMatrix;
}

template <typename PointT>
Eigen::MatrixXd computeCenteredCovarianceMatrix(typename pcl::PointCloud<PointT>::Ptr pointCloud1, typename pcl::PointCloud<PointT>::Ptr pointCloud2)	{
	//Covariance Matrix container, is a 3x3 matrix
	Eigen::MatrixXd covarianceMatrix(3, 3);
	//set all to 0
	covarianceMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	// Make sure all size are =
	if(pointCloud1->size() != pointCloud2->size())	{
		return covarianceMatrix;
	}
	// calculate covariance matrix
	for(int i = 0; i < pointCloud1->size(); i++)	{
		//calculate sub matrix
		Eigen::Vector3d temp1, temp2;
		//construct the 1x3 and 3x1 matrix
		temp1 << pointCloud1->points[i].x, pointCloud1->points[i].y, pointCloud1->points[i].z;
		temp2 << pointCloud2->points[i].x, pointCloud2->points[i].y, pointCloud2->points[i].z;

		covarianceMatrix += (temp1 * temp2.transpose());
	}

	return covarianceMatrix;
}

template <typename PointT>
void millimeterToMeter(PointT& point)	{
	point.x /= 1000.0;
	point.y /= 1000.0;
	point.z /= 1000.0;
}

template <typename PointT>
void millimeterToMeter(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		it->x /= 1000.0;
		it->y /= 1000.0;
		it->z /= 1000.0;
	}
}

template <typename PointT>
void meterToMillimeter(PointT& point)	{
	point.x *= 1000.0;
	point.y *= 1000.0;
	point.z *= 1000.0;
}

template <typename PointT>
void meterToMillimeter(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
	for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		it->x *= 1000.0;
		it->y *= 1000.0;
		it->z *= 1000.0;
	}
}



template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr applyTransformation(typename pcl::PointCloud<PointT>::Ptr pointCloud, 
														  Eigen::MatrixXf rotationMatrix, 
														  Eigen::MatrixXf translationVector)	{

	typename pcl::PointCloud<PointT>::Ptr pclPointCloud(new pcl::PointCloud<PointT>);
	// pclPointCloud = R * pointCloud + T
	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		Eigen::MatrixXf point(3, 1);
		// convert PCL points to Eigen Vector
		point << it->x, it->y, it->z;

		point = rotationMatrix * point + translationVector;

		pclPointCloud->push_back(PointT(*it));
		pclPointCloud->back().x = point(0,0);
		pclPointCloud->back().y = point(1,0);
		pclPointCloud->back().z = point(2,0);
	}

	return pclPointCloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr applyTranslation(typename pcl::PointCloud<PointT>::Ptr pointCloud, 
														  Eigen::MatrixXf translationVector)	{

	typename pcl::PointCloud<PointT>::Ptr pclPointCloud(new pcl::PointCloud<PointT>);
	// pclPointCloud = R * pointCloud + T
	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		Eigen::MatrixXf point(3, 1);
		// convert PCL points to Eigen Vector
		point << it->x, it->y, it->z;

		point = point + translationVector;

		pclPointCloud->push_back(PointT(*it));
		pclPointCloud->back().x = point(0,0);
		pclPointCloud->back().y = point(1,0);
		pclPointCloud->back().z = point(2,0);
	}

	return pclPointCloud;
}

template<typename PointT>
void removeTranslationalComponents(typename pcl::PointCloud<PointT>::Ptr pointCloud, pcl::PointXYZ centroid)	{
	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		it->x = it->x - centroid.x;
		it->y = it->y - centroid.y;
		it->z = it->z - centroid.z;
	}
}

template<typename PointT>
void getMinMax(typename pcl::PointCloud<PointT>::Ptr pointCloud, double& minX, double& maxX, double& minY, double& maxY, 
			   double& minZ, double& maxZ)	{
	minX = minY = minZ = LLONG_MAX;
	maxX = maxY = maxZ = LLONG_MIN;

	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		if(it->x < minX){
			minX = it->x;
		}
		if(it->x > maxX){
			maxX = it->x;
		}
		if(it->y < minY){
			minY = it->y;
		}
		if(it->y > maxY){
			maxY = it->y;
		}
		if(it->z < minZ){
			minZ = it->z;
		}
		if(it->z > maxZ){
			maxZ = it->z;
		}
	}
}

template<typename PointT>
void getMinMax(typename pcl::PointCloud<PointT>::Ptr pointCloud, std::pair<double, double>& minMaxX, 
			   std::pair<double, double>& minMaxY, std::pair<double, double>& minMaxZ)	{

	minMaxX.first = minMaxY.first = minMaxZ.first = LLONG_MAX;
	minMaxX.second = minMaxY.second = minMaxZ.second = LLONG_MIN;

	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		if(it->x < minMaxX.first){
			minMaxX.first = it->x;
		}
		if(it->x > minMaxX.second){
			minMaxX.second = it->x;
		}
		if(it->y < minMaxY.first){
			minMaxY.first = it->y;
		}
		if(it->y > minMaxY.second){
			minMaxY.second = it->y;
		}
		if(it->z < minMaxZ.first){
			minMaxZ.first = it->z;
		}
		if(it->z > minMaxZ.second){
			minMaxZ.second = it->z;
		}
	}
}

template<typename PointT>
std::map<double, std::pair<int, int> > computeDistanceBTWPoints(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
	std::map<double, std::pair<int, int> > distanceMap;
	double distance = 0.0;

	for(int i = 0; i < pointCloud->size() - 1; i++)	{
		for(int j = i + 1; j < pointCloud->size(); j++)	{
			distance = sqrt(pow(pointCloud->points[i].x - pointCloud->points[j].x, 2) +
							pow(pointCloud->points[i].y - pointCloud->points[j].y, 2) +
							pow(pointCloud->points[i].z - pointCloud->points[j].z, 2));

			distanceMap[distance] = std::pair<int, int>(i, j);
		}
	}

	return distanceMap;
}

/*
	Up/Down scale by given ratio
*/
template<typename PointT>
void scalePointCloud(typename pcl::PointCloud<PointT>::Ptr pointCloud, double scaleFactorX = 1.0, double scaleFactorY = 1.0,
					 double scaleFactorZ = 1.0)	{
	for(typename pcl::PointCloud<PointT>::iterator it = pointCloud->begin(); it != pointCloud->end(); it++)	{
		it->x *= scaleFactorX;
		it->y *= scaleFactorY;
		it->z *= scaleFactorZ;
	}
}

/*
	Up/Down Scale Point Cloud by point distance
*/
template<typename PointT>
double scalePointCloud(typename pcl::PointCloud<PointT>::Ptr srcCorrPoints,
					   typename pcl::PointCloud<PointT>::Ptr destCorrPoints,
					   typename pcl::PointCloud<PointT>::Ptr srcPoints)	{
	// Compute distance between each points
	std::map<double, std::pair<int, int> > srcDistance = computeDistanceBTWPoints<pcl::PointXYZRGB>(srcCorrPoints);
	std::map<double, std::pair<int, int> > destDistance = computeDistanceBTWPoints<pcl::PointXYZRGB>(destCorrPoints);

	#ifdef DEBUGGING
		std::clog << "Src Point Distance:\n";
		for(std::map<double, std::pair<int, int> >::iterator it = srcDistance.begin(); it != srcDistance.end(); it++)	{
			std::clog << "Distance: " << it->first << " with Index: " << it->second.first << " --- " << it->second.second << "\n";
		}
		std::clog << "\nDest Point Distance:\n";
		for(std::map<double, std::pair<int, int> >::iterator it = destDistance.begin(); it != destDistance.end(); it++)	{
			std::clog << "Distance: " << it->first << " with Index: " << it->second.first << " --- " << it->second.second << "\n";
		}
		std::clog << "\n\n";
	#endif

	// Compute the ratio for the furthest distance points
	double ratio = (destDistance.rbegin()->first) / (srcDistance.rbegin()->first);
	// scale the source correspondence point
	scalePointCloud<pcl::PointXYZRGB>(srcCorrPoints, ratio, ratio);
	scalePointCloud<pcl::PointXYZRGB>(srcPoints, ratio, ratio);

	return ratio;
}


#endif