#ifndef BISECTING_LINEAR_LINE_COMPLEX_REGISTRATION_H
	#define BISECTING_LINEAR_LINE_COMPLEX_REGISTRATION_H 
// Standard Libraries
#include <iostream>
#include <cmath>

// 3rd Parties Libraries
//PCL
#include <pcl/common/common_headers.h>

//Eigen
#include <Eigen/Dense>

//Local Libraries
#include "eigen_utils.h"
#include "point_cloud_utils.h"
#include "point_cloud_registration.h"

template <typename PointT>
class BisectorRegistration : public Registration<PointT>	{
	private:
		Eigen::MatrixXd g;
		Eigen::MatrixXd m;
		Eigen::MatrixXd W;
		Eigen::MatrixXd E;

		void initializeMatrix(int N)	{
			g = Eigen::MatrixXd::Zero(N, 3);
			m = Eigen::MatrixXd::Zero(N, 3);
			W = Eigen::MatrixXd::Zero(N, 1);
			E = Eigen::MatrixXd::Zero(N, 3);
		}

		Eigen::MatrixXd rodriguezEquation(Eigen::Vector3d axis, double angle)	{
			double theta = angle * M_PI / 180.0;
			double sinThetaDiv2 = sin(theta / 2.0),
				   cosThetaDiv2 = cos(theta / 2.0),
				   pow2SinTheta = pow(sinThetaDiv2, 2);

			Eigen::MatrixXd transformation = Eigen::MatrixXd::Zero(3, 3);

			transformation << 2.0 * (pow(axis(1), 2) - 1) * pow2SinTheta + 1,
							  2.0 * sinThetaDiv2 * (axis(2) * axis(1) * sinThetaDiv2 - axis(3) * cosThetaDiv2),
							  2.0 * sinThetaDiv2 * (axis(3) * axis(1) * sinThetaDiv2 + axis(2) * cosThetaDiv2),
							  2.0 * sinThetaDiv2 * (axis(1) * axis(2) * sinThetaDiv2 + axis(3) * cosThetaDiv2),
							  2.0 * (pow(axis(2), 2) - 1) * pow2SinTheta + 1,
							  2.0 * sinThetaDiv2 * (axis(3) * axis(2) * sinThetaDiv2 - axis(1) * cosThetaDiv2),
							  2.0 * sinThetaDiv2 * (axis(1) * axis(3) * sinThetaDiv2 - axis(2) * cosThetaDiv2),
							  2.0 * sinThetaDiv2 * (axis(2) * axis(3) * sinThetaDiv2 + axis(1) * cosThetaDiv2),
							  2.0 * (pow(axis(3), 2) - 1) * pow2SinTheta + 1;

			return transformation;
		}

	protected:

	public:
		BisectorRegistration() : Registration<PointT>()	{

		}
		~BisectorRegistration()	{

		}

		bool computeTransformation(typename pcl::PointCloud<PointT>::Ptr srcCorrPointCloud, typename pcl::PointCloud<PointT>::Ptr dstCorrPointCloud)	{
			if(srcCorrPointCloud == NULL || dstCorrPointCloud == NULL ||
			   srcCorrPointCloud->size() != dstCorrPointCloud->size())	{
				return false;
			}

			Eigen::MatrixXd mTrans, ETrans, F,
							linesP, linesQ, crossGP, crossGQ, angles,
							axis1;
			Eigen::Vector3d srcPoint, dstPoint, srcPoint2, dstPoint2,
							c, cBar, s, sBar, displacement;
			double pbblc = 0.0, angle1 = 0.0, angle2 = 0.0, d = 0.0,
				   phi1 = 0.0;

			int N = srcCorrPointCloud->size();

			initializeMatrix(N);

			
			for(int i = 0; i < N; ++i)	{
				// convert PCL point to Eigen Vector3d
				srcPoint << srcCorrPointCloud->points[i].x, srcCorrPointCloud->points[i].y, srcCorrPointCloud->points[i].z;
				dstPoint << dstCorrPointCloud->points[i].x, dstCorrPointCloud->points[i].y, dstCorrPointCloud->points[i].z;

				g.row(i) = (srcPoint + dstPoint) / 2.0;
				m.row(i) = dstPoint - srcPoint;
				W.row(i) = m.row(i).dot(g.row(i));
				E.row(i) = m.row(i).cross(g.row(i));
			}

			mTrans = m.transpose();
			ETrans = E.transpose();

			cBar = -1 * ((mTrans * m).inverse()) * mTrans * W;
			F = Eigen::MatrixXd::Zero(N, 1);

			for(int i = 0; i < N; ++i)	{
				F.row(i) = (g.row(i) + cBar.transpose()).dot(g.row(i));
			}

			c = -1 * ((ETrans * E).inverse()) * ETrans * F;
			pbblc = c.dot(cBar) / c.dot(c);

			s = c;
			sBar = cBar - pbblc * c;
			s = s / s.norm();

			linesP = Eigen::MatrixXd::Zero(N - 1, 3);
			linesQ = Eigen::MatrixXd::Zero(N - 1, 3);
			crossGP = Eigen::MatrixXd::Zero(N - 1, 3);
			crossGQ = Eigen::MatrixXd::Zero(N - 1, 3);
			angles = Eigen::MatrixXd::Zero(N - 1, 1);

			for(int i = 0; i < N; ++i)	{
				// convert PCL point to Eigen Vector3d
				srcPoint << srcCorrPointCloud->points[i].x, srcCorrPointCloud->points[i].y, srcCorrPointCloud->points[i].z;
				dstPoint << dstCorrPointCloud->points[i].x, dstCorrPointCloud->points[i].y, dstCorrPointCloud->points[i].z;

				d = d + (double)((dstPoint - srcPoint) * s / s.norm());
			}

			d = d / N;

			for(int i = 0; i < N - 1; ++i)	{
				// convert PCL point to Eigen Vector3d
				srcPoint << srcCorrPointCloud->points[i].x, srcCorrPointCloud->points[i].y, srcCorrPointCloud->points[i].z;
				dstPoint << dstCorrPointCloud->points[i].x, dstCorrPointCloud->points[i].y, dstCorrPointCloud->points[i].z;
				srcPoint2 << srcCorrPointCloud->points[i + 1].x, srcCorrPointCloud->points[i + 1].y, srcCorrPointCloud->points[i + 1].z;
				dstPoint2 << dstCorrPointCloud->points[i + 1].x, dstCorrPointCloud->points[i + 1].y, dstCorrPointCloud->points[i + 1].z;

				linesP.row(i) = (srcPoint2 - srcPoint) / (srcPoint2 - srcPoint).norm();
				linesQ.row(i) = (dstPoint2 - dstPoint) / (dstPoint2 - dstPoint).norm();
				crossGP.row(i) = linesP.row(i).cross(s);
				crossGQ.row(i) = linesQ.row(i).cross(s);
				crossGP.row(i) = crossGP.row(i) / crossGP.row(i).norm();
				crossGQ.row(i) = crossGQ.row(i) / crossGQ.row(i).norm();
				angles.row(i) = (acos(crossGP.row(i).dot(crossGQ.row(i)))) * 180.0 / M_PI;
				angle1 = angle1 + crossGP.row(i).dot(crossGQ.row(i));
				angle2 = angle2 + crossGP.row(i).norm() * crossGQ.row(i).norm();
			}

			phi1 = acos(angle1 / angle2) * 180.0 / M_PI;
			angle1 = phi1;

			Registration<PointT>::translationVector = -d * s;

			Registration<PointT>::rotationMatrix = rodriguezEquation(s, angle1);

			return true;
		}

		bool pointTransformation(PointT& point)	{

		}

		bool pointCloudTransformation(typename pcl::PointCloud<PointT>::Ptr pointCloud)	{

		}

};

	
#endif