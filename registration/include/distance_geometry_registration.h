/*
	* Software License Agreement (BSD License)
	*
	*  Point Cloud Library (PCL) - www.pointclouds.org
	*  Copyright (c) 2010-2011, Willow Garage, Inc.
	*
	*  All rights reserved.
	*
	*  Redistribution and use in source and binary forms, with or without
	*  modification, are permitted provided that the following conditions
	*  are met:
	*
	*   * Redistributions of source code must retain the above copyright
	*     notice, this list of conditions and the following disclaimer.
	*   * Redistributions in binary form must reproduce the above
	*     copyright notice, this list of conditions and the following
	*     disclaimer in the documentation and/or other materials provided
	*     with the distribution.
	*   * Neither the name of Willow Garage, Inc. nor the names of its
	*     contributors may be used to endorse or promote products derived
	*     from this software without specific prior written permission.
	*
	*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	*  POSSIBILITY OF SUCH DAMAGE.
	*

	Author: Hiu Hong Yu
*/
#ifndef POINT_CLOUD_DISTANCE_GEOMETRY_REGISTRATION_UTILS_H
	#define POINT_CLOUD_DISTANCE_GEOMETRY_REGISTRATION_UTILS_H 
// Standard Libraries
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <iomanip>
#include <thread>

// 3rd Parties Libraries
//PCL
#include <pcl/common/common_headers.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>

//Local Libraries
#include "eigen_utils.h"
#include "point_cloud_utils.h"
#include "point_cloud_registration.h"


//Enum
enum THREAD_MODE	{
	THREAD_ENABLE,
	THREAD_DISABLE
};

//MACRO
#define NUM_REF_POINTS 4


template <typename PointT>
class DistanceGeometryRegistration : public Registration<PointT>	{
	private:
		const Eigen::Vector3d refPoint[NUM_REF_POINTS];
		// Initialize matrix default values to be all 0
		Eigen::MatrixXd referenceDistanceMatrix;
		Eigen::MatrixXd inverseRDM;
		Eigen::MatrixXd identityMatrix;
		Eigen::VectorXd h;
		Eigen::VectorXd en;

		void initializeUMatrix(Eigen::MatrixXd& UMatrix, typename pcl::PointCloud<PointT>::Ptr points)	{
			Eigen::Vector3d temp;
			for(int i = 0; i < points->size(); i++)	{
				// convert PCL Point into 3 element Vector
				temp << points->points[i].x, points->points[i].y, points->points[i].z; 
				UMatrix(0, i) = (refPoint[0] - temp).dot((refPoint[0] - temp).transpose());
				UMatrix(1, i) = (refPoint[1] - temp).dot((refPoint[1] - temp).transpose());
				UMatrix(2, i) = (refPoint[2] - temp).dot((refPoint[2] - temp).transpose());
				UMatrix(3, i) = (refPoint[3] - temp).dot((refPoint[3] - temp).transpose());
				UMatrix(4, i) = 1.0;
			}
		}

		Eigen::VectorXd baryCentric2Cartesian(Eigen::VectorXd Up)	{
			// Solving linearing equation ax = b
			Eigen::MatrixXd a(4, 6);
			Eigen::Vector4d b;

			a << 1.0, 1.0, 1.0, -2.0 * refPoint[0](0), -2.0 * refPoint[0](1), -2.0 * refPoint[0](2),
				 1.0, 1.0, 1.0, -2.0 * refPoint[1](0), -2.0 * refPoint[1](1), -2.0 * refPoint[1](2),
				 1.0, 1.0, 1.0, -2.0 * refPoint[2](0), -2.0 * refPoint[2](1), -2.0 * refPoint[2](2), 
				 1.0, 1.0, 1.0, -2.0 * refPoint[3](0), -2.0 * refPoint[3](1), -2.0 * refPoint[3](2);

			b << Up(0) - pow(refPoint[0](0), 2) - pow(refPoint[0](1), 2) - pow(refPoint[0](2), 2),
				 Up(1) - pow(refPoint[1](0), 2) - pow(refPoint[1](1), 2) - pow(refPoint[1](2), 2),
				 Up(2) - pow(refPoint[2](0), 2) - pow(refPoint[2](1), 2) - pow(refPoint[2](2), 2),
				 Up(3) - pow(refPoint[3](0), 2) - pow(refPoint[3](1), 2) - pow(refPoint[3](2), 2);

			// mldivide
			return a.fullPivHouseholderQr().solve(b);
		}

	protected:
	public:
		/*
			Constructor/Destructor
		*/
		DistanceGeometryRegistration() : Registration<PointT>(),
										 refPoint({Eigen::Vector3d(2.0, 3.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0),
												   Eigen::Vector3d(5.0, 0.0, 0.0), Eigen::Vector3d(3.0, -1.0, 4.0)})	{
			
			Registration<PointT>::transformationMatrix = Eigen::MatrixXd::Zero(5, 5);

			// init class variables
			referenceDistanceMatrix = Eigen::MatrixXd::Zero(5, 5);
			inverseRDM = Eigen::MatrixXd::Zero(5, 5);
			identityMatrix = Eigen::MatrixXd::Identity(5, 5);
			
			// Init Vector
			h = Eigen::VectorXd::Ones(NUM_REF_POINTS + 1);
			en = Eigen::VectorXd::Zero(NUM_REF_POINTS + 1);

			initialize();
		}

		~DistanceGeometryRegistration()	{

		}

		/*
			Class Methods
		*/
		void initialize()	{
			// Initialize referenceMatrix
			// compute referenceDistanceMatrix
			// set non-zero element for Au matrix
			// Since referenceDistanceMatrix = Au + transpose(Au), we will not create Au
			referenceDistanceMatrix(0, 1) = (refPoint[0] - refPoint[1]).dot((refPoint[0] - refPoint[1]).transpose());
			referenceDistanceMatrix(0, 2) = (refPoint[0] - refPoint[2]).dot((refPoint[0] - refPoint[2]).transpose());
			referenceDistanceMatrix(0, 3) = (refPoint[0] - refPoint[3]).dot((refPoint[0] - refPoint[3]).transpose());
			referenceDistanceMatrix(0, 4) = 1.0;
			referenceDistanceMatrix(1, 2) = (refPoint[1] - refPoint[2]).dot((refPoint[1] - refPoint[2]).transpose());
			referenceDistanceMatrix(1, 3) = (refPoint[1] - refPoint[3]).dot((refPoint[1] - refPoint[3]).transpose());
			referenceDistanceMatrix(1, 4) = 1.0;
			referenceDistanceMatrix(2, 3) = (refPoint[2] - refPoint[3]).dot((refPoint[2] - refPoint[3]).transpose());
			referenceDistanceMatrix(2, 4) = 1.0;
			referenceDistanceMatrix(3, 4) = 1.0;
			// referenceDistanceMatrix = Au + transpost(Au)
			// using eval will force Eigen to eval the express first to avoid aliasing problem
			referenceDistanceMatrix = (referenceDistanceMatrix + referenceDistanceMatrix.transpose()).eval();
			inverseRDM = referenceDistanceMatrix.inverse();
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << 
							 "refPoint[0]:\n" << refPoint[0] << "\n\n" <<
							 "refPoint[1]:\n" << refPoint[1] << "\n\n" <<
							 "refPoint[2]:\n" << refPoint[2] << "\n\n" <<
							 "refPoint[3]:\n" << refPoint[3] << "\n\n" <<
							 "referenceDistanceMatrix:\n" << referenceDistanceMatrix << "\n\n" << 
							 "inverseRDM:\n" << inverseRDM << "\n\n";
			#endif

			//Initialize h
			h(NUM_REF_POINTS) = 0.0;
			#ifdef DEBUGGING
				std::cout << "h:\n" << h << "\n\n";
			#endif
			//Initialize en
			en(NUM_REF_POINTS) = 1.0;
			#ifdef DEBUGGING
				std::cout << "en:\n" << en << "\n\n";
			#endif
		}

		// Computer transformation matrix that transform srcCorrPointCloud to destCorrPointCloud
		bool computeTransformation(typename pcl::PointCloud<PointT>::Ptr srcCorrPointCloud, typename pcl::PointCloud<PointT>::Ptr destCorrPointCloud)	{
			if(srcCorrPointCloud == NULL || destCorrPointCloud == NULL)	{
				return false;
			}
			// Convert all correspondence points to MM
			millimeterToMeter<PointT>(srcCorrPointCloud);
			millimeterToMeter<PointT>(destCorrPointCloud);

			// Local variables and constants
			int N = destCorrPointCloud->size();
			double kp = 0.0, kq = 0.0;
			Eigen::VectorXd hN, upAvg, uqAvg, up, xp, uq, xq, tmp, lv, pq;
			Eigen::MatrixXd Up, Uq, E, F, Ar, Fr, V, D, rootD, L, X01, lambda;

			/*
				Initialize vector/matrix
			*/
			// Init Vector
			hN = Eigen::VectorXd::Ones(N);
			upAvg = Eigen::VectorXd::Zero(5);
			uqAvg = Eigen::VectorXd::Zero(5);
			up = Eigen::VectorXd::Zero(5);
			xp = Eigen::VectorXd::Zero(5);
			uq = Eigen::VectorXd::Zero(5);
			xq = Eigen::VectorXd::Zero(5);
			tmp = Eigen::VectorXd::Zero(5);
			lv = Eigen::VectorXd::Zero(4);
			pq = Eigen::VectorXd::Zero(5);
			// Init Matrix
			Up = Eigen::MatrixXd::Zero(NUM_REF_POINTS + 1, N);
			Uq = Eigen::MatrixXd::Zero(NUM_REF_POINTS + 1, N);
			E = Eigen::MatrixXd::Zero(5, 5);
			F = Eigen::MatrixXd::Zero(5, 5);
			Ar = Eigen::MatrixXd::Zero(4, 4);
			Fr = Eigen::MatrixXd::Zero(4, 4);
			V = Eigen::MatrixXd::Zero(4, 4);
			D = Eigen::MatrixXd::Zero(4, 4);
			rootD = Eigen::MatrixXd::Zero(4, 4);
			L = Eigen::MatrixXd::Zero(4, 4);
			X01 = Eigen::MatrixXd::Zero(5, 5);
			lambda = Eigen::MatrixXd::Zero(5, 5);

			#ifdef DEBUGGING
				std::cout << "hN:\n" << hN << "\n\n";
			#endif
			// Initial Up and Uq matrix (5xN)
			// Up = The points in QPrime (denoted by tarPointCloud)
			// Up Matrix
			initializeUMatrix(Up, destCorrPointCloud);
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "Up:\n" << Up << "\n\n";
			#endif
			// Uq Matrix
			initializeUMatrix(Uq, srcCorrPointCloud);
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "Uq:\n" << Uq << "\n\n";
			#endif
			// Compute Up and Uq average
			upAvg = Up * hN / N;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "upAvg:\n" << upAvg << "\n\n";
			#endif
			uqAvg = Uq * hN / N;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "uqAvg:\n" << uqAvg << "\n\n";
			#endif
			// Compute kp and kq
			kp = (N / 2.0) * upAvg.transpose() * inverseRDM * upAvg;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "kp:\n" << kp << "\n\n";
			#endif
			kq = (N / 2.0) * uqAvg.transpose() * inverseRDM * uqAvg;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "kq:\n" << kq << "\n\n";
			#endif
			// Compute the centroids
			up = upAvg - (kp / N) * h;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "up:\n" << up << "\n\n";
			#endif
			// xp = inverseRDM * up;
			// #ifdef DEBUGGING
			// 	std::cout << std::setprecision(10) << "xp:\n" << xp << "\n\n";
			// #endif
			uq = uqAvg - (kq / N) * h;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "uq:\n" << uq << "\n\n";
			#endif
			xq = inverseRDM * uq;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "xq:\n" << xq << "\n\n";
			#endif
			// Compute E matrix
			E = Up * Uq.transpose() - N * up * uq.transpose();
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "E:\n" << E << "\n\n";
			#endif
			// Compute F matrix
			F = inverseRDM * E * inverseRDM;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "F:\n" << F << "\n\n";
			#endif
			// Remove last row from referenceDistanceMatrix and F matrix
			Ar = referenceDistanceMatrix;
			removeRow(Ar, 4);
			removeColumn(Ar, 4);
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "Ar:\n" << Ar << "\n\n";
			#endif
			Fr = F;
			removeRow(Fr, 4);
			removeColumn(Fr, 4);
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "Fr:\n" << Fr << "\n\n";
			#endif
			// Diagonalizing
			eig(Fr.transpose() * Ar * Fr * Ar, V, D);
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "V:\n" << V << "\n\n" << "D:\n" << D << "\n\n";
			#endif
			// Square roots of elements of D
			rootD = D.sqrt();
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "rootD:\n" << rootD << "\n\n";
			#endif
			// Lambda-r * Ar matrix
			L = -V * rootD * V.inverse();
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "L:\n" << L << "\n\n";
			#endif
			// lambda-v column vector
			tmp = E.inverse() * up;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "tmp:\n" << tmp << "\n\n" << "inverseE:\n" << E.inverse() << "\n\n";
			#endif
			lv = kq * (Eigen::Vector4d(xq(0), xq(1), xq(2), xq(3)) - L * Eigen::Vector4d(tmp(0), tmp(1), tmp(2), tmp(3)));
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "lv:\n" << lv << "\n\n";
			#endif
			/*
				Construct tempVector
				Where tempVector = transpose(lv) * Ar
			*/
			Eigen::VectorXd tempVector(4);
			tempVector = lv.transpose() * Ar;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "tempVector:\n" << tempVector << "\n\n";
			#endif
			/*	Construct tempMatrix
				tempMatrix representing a 5x5 matrix, where it is constructed by 
				L00 L01 L02 L03 0
				L10 L11 L12 L13 0
				L20 L21 L22 L23 0
				L30 L31 L32 L33 0
				transpose(lv)*Ar kq	
			*/
			Eigen::MatrixXd tempMatrix(5, 5);
			tempMatrix << L(0, 0), L(0, 1), L(0, 2), L(0, 3), 0.0, L(1, 0), L(1, 1), L(1, 2), L(1, 3), 0.0,
						  L(2, 0), L(2, 1), L(2, 2), L(2, 3), 0.0, L(3, 0), L(3, 1), L(3, 2), L(3, 3), 0.0,
						  tempVector(0), tempVector(1), tempVector(2), tempVector(3), kq;
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "tempMatrix:\n" << tempMatrix << "\n\n";
			#endif
			Eigen::MatrixXd transposeF = F.transpose();
			Eigen::MatrixXd inverseTransF = transposeF.inverse();
			// inverseTransF(4, 4) = 0.0;
			Registration<PointT>::transformationMatrix = inverseTransF * tempMatrix * (identityMatrix - xq * h.transpose()) + up * h.transpose();
			#ifdef DEBUGGING
				std::cout << std::setprecision(10) << "transformationMatrix:\n" << Registration<PointT>::transformationMatrix << "\n\n";
			#endif

			// Lagrange multiplier matrix
			// X01 = inverseRDM * Registration<PointT>::transformationMatrix;
			// #ifdef DEBUGGING
			// 	std::cout << std::setprecision(10) << "X01:\n" << X01 << "\n\n";
			// #endif

			// lambda = inverseRDM * Registration<PointT>::transformationMatrix.transpose() * F;
			// #ifdef DEBUGGING
			// 	std::cout << std::setprecision(10) << "lambda:\n" << lambda << "\n\n";
			// #endif

			// Residue
			// R = L.trace() + kp + kq;
			// #ifdef DEBUGGING
			// 	std::cout << std::setprecision(10) << "R:\n" << R << "\n\n";
			// #endif

			// Distances of the points in QPrime to their homologous points in Q
			// pq = getDiagonalElements(Up.transpose() * inverseRDM * Registration<PointT>::transformationMatrix * inverseRDM * Uq, 5);
			// #ifdef DEBUGGING
			// 	std::cout << std::setprecision(10) << "pq:\n" << pq << "\n\n";
			// #endif
			// Convert all points back to millimeter
			meterToMillimeter<PointT>(srcCorrPointCloud);
			meterToMillimeter<PointT>(destCorrPointCloud);
			// Final step: Since the when performing transformation, the equation is:
			// TransformationMatrix*InverseReferenceMatrix*distanceCoordinate
			// We will make TransformationMatrix = TransformationMatrix * InverseReferenceMatrix
			// to save computational power
			Registration<PointT>::transformationMatrix = Registration<PointT>::transformationMatrix * inverseRDM;
			// If transformationMatrix is NaN matrix, then transformationMatrix computation FAILED
			return !isNanMatrix(Registration<PointT>::transformationMatrix);
		}

		bool pointCloudTransformation(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
			if(pointCloud == NULL || isNanMatrix(Registration<PointT>::transformationMatrix))	{
				return false;
			}
			// Convert all points to meter
			millimeterToMeter<PointT>(pointCloud);

			// For every point from point cloud 2, map it onto point cloud 1 coordinate
			Eigen::Vector3d point;
			Eigen::VectorXd distanceCoordinate(5), registrationVector(5);
			for(int i = 0; i < pointCloud->size(); i++)	{
				// Point Vector
				point << pointCloud->points[i].x, pointCloud->points[i].y, pointCloud->points[i].z;
				// Convert from cartesian coordinate to distance coordinate
				// norm(pointnorm(srcPointCloud - a) = distance between ith point in point cloud to a
				distanceCoordinate << (refPoint[0] - point).dot((refPoint[0] - point).transpose()),
									  (refPoint[1] - point).dot((refPoint[1] - point).transpose()),
									  (refPoint[2] - point).dot((refPoint[2] - point).transpose()),
									  (refPoint[3] - point).dot((refPoint[3] - point).transpose()),
									  1.0;

				// Perform registration
				registrationVector = (Registration<PointT>::transformationMatrix * distanceCoordinate).transpose();
				// Convert distance coordinate to cartesian coordinate
				Eigen::VectorXd temp = baryCentric2Cartesian(registrationVector);
				pointCloud->points[i].x = temp(3); 
				pointCloud->points[i].y = temp(4); 
				pointCloud->points[i].z = temp(5); 
			}

			// Convert all points to millimeter
			meterToMillimeter<PointT>(pointCloud);

			return true;
		}

		void pointTransformation(PointT& point)	{
			// Convert all points to meter
			millimeterToMeter<PointT>(point);

			Eigen::Vector3d eigenPoint;
			Eigen::VectorXd distanceCoordinate(5), registrationVector(5);

			// Point Vector
			eigenPoint << point.x, point.y, point.z;
			// Convert from cartesian coordinate to distance coordinate
			// norm(pointnorm(srcPointCloud - a) = distance between ith point in point cloud to a
			distanceCoordinate << (refPoint[0] - eigenPoint).dot((refPoint[0] - eigenPoint).transpose()),
								  (refPoint[1] - eigenPoint).dot((refPoint[1] - eigenPoint).transpose()),
								  (refPoint[2] - eigenPoint).dot((refPoint[2] - eigenPoint).transpose()),
								  (refPoint[3] - eigenPoint).dot((refPoint[3] - eigenPoint).transpose()),
								  1.0;

			// Perform registration
			registrationVector = (Registration<PointT>::transformationMatrix * distanceCoordinate).transpose();
			// Convert distance coordinate to cartesian coordinate
			Eigen::VectorXd temp = baryCentric2Cartesian(registrationVector);
			point.x = temp(3); 
			point.y = temp(4); 
			point.z = temp(5); 

			// Convert all points to millimeter
			meterToMillimeter<PointT>(point);
		}

		// Compute the registration error
		// This return the absolute error in millimeter
		double estimateError(typename pcl::PointCloud<PointT>::Ptr srcPointCloud,
							 typename pcl::PointCloud<PointT>::Ptr tarPointCloud)	{
			double error = 0.0;
			for(int i = 0; i < tarPointCloud->size(); i++)	{
				error += (Eigen::Vector3d(srcPointCloud->points[i].x, srcPointCloud->points[i].y, srcPointCloud->points[i].z) - 
						  Eigen::Vector3d(tarPointCloud->points[i].x, tarPointCloud->points[i].y, tarPointCloud->points[i].z)).norm();
			}

			return error / (double)tarPointCloud->size();
		}

		bool pointCloudRegistration(pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcCorrPoints, 
									  pcl::PointCloud<pcl::PointXYZRGB>::Ptr destCorrPoints,
									  pcl::PointCloud<pcl::PointXYZRGB>::Ptr srcPoints,
									  THREAD_MODE threadMode)	{
			int threadAmount = 1;
			/*
				Compute the scaling between point cloud
			*/
			//pick the farest 2 points
			std::map<double, std::pair<int, int> > srcDistance = computeDistanceBTWPoints<pcl::PointXYZRGB>(srcCorrPoints);
			std::map<double, std::pair<int, int> > destDistance = computeDistanceBTWPoints<pcl::PointXYZRGB>(destCorrPoints);
			
			#ifdef DEBUGGING
				for(std::map<double, std::pair<int, int> >::iterator it = srcDistance.begin(); it != srcDistance.end(); it++)	{
					std::cout << "Distance: " << it->first << " with Index: " << it->second.first << " --- " << it->second.second << "\n";
				}
				std::cout << "\n\n";
				for(std::map<double, std::pair<int, int> >::iterator it = destDistance.begin(); it != destDistance.end(); it++)	{
					std::cout << "Distance: " << it->first << " with Index: " << it->second.first << " --- " << it->second.second << "\n";
				}
			#endif

			double ratio = (destDistance.rbegin()->first) / (srcDistance.rbegin()->first);
			
			#ifdef DEBUGGING
				std::cout << "Scale Ratio: " << ratio << "\n";
			#endif

			// scale the source correspondence point
			scalePointCloud<pcl::PointXYZRGB>(srcCorrPoints, ratio, ratio);
			scalePointCloud<pcl::PointXYZRGB>(srcPoints, ratio, ratio);

			/*
				Point Cloud Registration
			*/
			//Compute the transformation matrix
			if(!computeTransformation(srcCorrPoints, destCorrPoints))	{
				return false;
			}

			// Determine thread amount
			if(threadMode == THREAD_ENABLE)	{
				for(threadAmount = 9; threadAmount > 0; threadAmount--)	{
					if((srcPoints->size() % threadAmount) == 0)	{
						break;
					}
				}
			}

			/*
				Apply transformation to point cloud using thread
			*/
			if(threadMode == THREAD_ENABLE)	{
				std::thread transformationThreads[threadAmount]; 
				for(int i = 0; i < srcPoints->size(); i += threadAmount)	{
					for(int t = 0; t < threadAmount; t++)	{
						transformationThreads[t] = std::thread(&DistanceGeometryRegistration<pcl::PointXYZRGB>::pointTransformation, 
															   this, std::ref(srcPoints->points[i + t]));
					}

					for(int t = 0; t < threadAmount; t++)	{
						transformationThreads[t].join();
					}
				}
			}
			else 	{
				pointCloudTransformation(srcPoints);
			}

			return true;
		}
};


#endif