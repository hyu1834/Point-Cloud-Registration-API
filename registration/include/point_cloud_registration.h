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
#ifndef POINT_CLOUD_REGISTRATION_H
	#define POINT_CLOUD_REGISTRATION_H 
// Standard Libraries
#include <iostream>
#include <cmath>

// 3rd Parties Libraries
//PCL
#include <pcl/common/common_headers.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>

//Local Libraries
#include "point_cloud_utils.h"

template <typename PointT>
class Registration	{
	private:
	protected:
		Eigen::MatrixXd rotationMatrix;
		Eigen::Vector3d translationVector;
		Eigen::MatrixXd transformationMatrix;
		
	public:
		/*
			Constructor/Destructor
		*/
		Registration()	{
			rotationMatrix = Eigen::MatrixXd::Zero(3, 3);
			translationVector = Eigen::Vector3d::Zero(3);
			transformationMatrix = Eigen::MatrixXd::Zero(4, 4);
		}

		~Registration()	{
			
		}

		/*
			Getter/Setter
		*/
		Eigen::MatrixXd getRotationMatrix()	{
			return rotationMatrix;
		}

		Eigen::Vector3d getTranslationMatrix()	{
			return translationVector;
		}

		/*
			Non-Template Class Methods
		*/


		/*
			Template Class Methods
		*/
		virtual bool computeTransformation()	{
			return true;
		}

		virtual bool pointCloudTransformation(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{
			if(pointCloud == NULL)	{
				return false;
			}			

			for(int i = 0; i < pointCloud->size(); i++)	{
				Eigen::Vector3d temp(pointCloud->points[i].x, pointCloud->points[i].y, pointCloud->points[i].z);
				temp = (rotationMatrix * temp) + translationVector;

				pointCloud->points[i].x = temp(0);
				pointCloud->points[i].y = temp(1); 
				pointCloud->points[i].z = temp(2);  
			}

			return true;
		}
};

#endif