#include "eigen_utils.h"

// removeRow by Andrew from stack overflow of the follow post
// http://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)	{
	unsigned int numRows = matrix.rows()-1;
	unsigned int numCols = matrix.cols();

	if( rowToRemove < numRows )
		matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

	matrix.conservativeResize(numRows,numCols);
}

// removecolumn by Andrew from stack overflow of the follow post
// http://stackoverflow.com/questions/13290395/how-to-remove-a-certain-row-or-column-while-using-eigen-library-c
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)	{
	unsigned int numRows = matrix.rows();
	unsigned int numCols = matrix.cols()-1;

	if( colToRemove < numCols )
		matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

	matrix.conservativeResize(numRows,numCols);
}

void initVector(Eigen::VectorXd& vec, double defaultValue, int size)   {
	for(int i = 0; i < size; i++)   {
		vec[i] = defaultValue;
	}
}

Eigen::VectorXd getDiagonalElements(Eigen::MatrixXd matrix, int size)  {
	Eigen::VectorXd diagonalElements(size);
	for(int i = 0; i < size; i++)   {
		diagonalElements[i] = matrix(i, i);
	}
	return diagonalElements;
}

// http://stackoverflow.com/questions/31041921/how-to-get-rank-of-a-matrix-in-eigen-library
int getMatrixRank(Eigen::MatrixXd matrix)   {
	Eigen::FullPivLU<Eigen::MatrixXd> luA(matrix);
	return luA.rank();
}

// http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix, double epsilon)  {
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(matrix.cols(), matrix.rows()) * svd.singularValues().array().abs()(0);
	return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

// eig follow the same way how Matlab is output it result
// where the eigenvalue is sorted in desending order 
// and eigenvector is sorted according to the eigenvalues
void eig(Eigen::MatrixXd A, Eigen::MatrixXd& V, Eigen::MatrixXd& D)   {
	// std::cout << "A:\n" << A << "\n\n";
	// Solve for the Eigenvalues and Eigenvector
	// Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(A);
	Eigen::ComplexEigenSolver<Eigen::MatrixXd> eigensolver(A);
	// std::cout << "V:\n" << eigensolver.eigenvectors().real() << "\n\nD:\n" << eigensolver.eigenvalues().real() << "\n\n";
	// Sort Eigenvector + Eigenvalue in desending order:
	// largest eigenvalue= most left, smallest eigenvalue = most right
	int dimension = eigensolver.eigenvalues().real().size(), temp;
	int* index = new int[dimension];
	for(int i = 0; i < dimension; i++)	{
		index[i] = i;
	}
	// Insertion sort
	int i, j;
	for(j = 1; j < dimension; j++)	{
		int key = index[j];
		for(i = j - 1; (i >= 0) && (eigensolver.eigenvalues().real()[i] < eigensolver.eigenvalues().real()[j]); i--)	{
			index[i + 1] = index[i];
		}
		index[i + 1] = key;
	}
	// Construct V and D matrix
	for(int i = 0; i < dimension; i++)	{
		V.col(i) = eigensolver.eigenvectors().col(index[i]).real();
		if(eigensolver.eigenvalues().real()[index[i]] < 0.0)	{
			D(i, i) = 0.0;
		}
		else 	{
			D(i, i) = eigensolver.eigenvalues().real()[index[i]];
		}
	}
 	delete index;
}

bool isNanMatrix(Eigen::MatrixXd A)	{
	for(int i = 0; i < A.rows(); i++)	{
		for(int j = 0; j < A.cols(); j++)	{
			if(std::isnan(A.row(i)[j]))	{
				return true;
			}
		}
	}
	return false;
}