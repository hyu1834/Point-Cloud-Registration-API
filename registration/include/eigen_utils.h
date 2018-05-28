#ifndef EIGEN_UTILS_H
	#define EIGEN_UTILS_H
// Standard Libraries
#include <iostream>
#include <iomanip>

// 3rd Parties Libraries
//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <unsupported/Eigen/MatrixFunctions>

//Local Libraries


// removeRow by Andrew from stack overflow of the follow post
// http://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);

// removecolumn by Andrew from stack overflow of the follow post
// http://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
void initVector(Eigen::VectorXd& vec, double defaultValue, int size);
Eigen::VectorXd getDiagonalElements(Eigen::MatrixXd matrix, int size);
int getMatrixRank(Eigen::MatrixXd matrix);
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix, double epsilon = std::numeric_limits<double>::epsilon());
void eig(Eigen::MatrixXd A, Eigen::MatrixXd& V, Eigen::MatrixXd& D);
bool isNanMatrix(Eigen::MatrixXd A);


#endif