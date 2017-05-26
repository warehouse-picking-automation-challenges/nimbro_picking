// Test different solvers on mtx file
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>

#ifdef KLU_SUPPORT
#include "../eigen_klu.h"
#endif

#ifdef EIGEN_UMFPACK_SUPPORT
#include "UmfPackSupport.h"
#endif

#include <chrono>

template<class Solver>
void testSolver(const Eigen::SparseMatrix<double>& A, const Eigen::VectorXd& b)
{
	std::cout << "\n\nSolving with solver " << typeid(Solver).name() << "\n";

	Solver solver;

	auto t0 = std::chrono::high_resolution_clock::now();

	solver.compute(A);
	auto x = solver.solve(b);

	auto t1 = std::chrono::high_resolution_clock::now();

	std::cout << "... took " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms.\n";

	Eigen::VectorXd residual = A*x - b;

	printf("residual norm: %e\n", residual.norm());
}

int main(int argc, char** argv)
{
	if(argc <= 1)
	{
		fprintf(stderr, "Usage: solve_test <prefix>\n");
	}

	Eigen::SparseMatrix<double> A;
	Eigen::VectorXd b;

	A.makeCompressed();

	std::string prefix = argv[1];
	Eigen::loadMarket(A, prefix + ".mtx");
	Eigen::loadMarketVector(b, prefix + "_b.mtx");

#ifdef KLU_SUPPORT
	testSolver<Eigen::KLU<Eigen::SparseMatrix<double>>>(A, b);
#endif
#ifdef EIGEN_UMFPACK_SUPPORT
	testSolver<Eigen::UmfPackLU<Eigen::SparseMatrix<double>>>(A, b);
#endif

	return 0;
}
