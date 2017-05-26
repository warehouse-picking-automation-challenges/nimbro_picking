// Eigen wrapper for the suitesparse KLU solver
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef EIGEN_KLU_H
#define EIGEN_KLU_H

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <iostream>

#include <klu.h>

namespace Eigen
{

template<typename _MatrixType>
class KLU : internal::noncopyable
{
public:
	typedef _MatrixType MatrixType;
	typedef typename MatrixType::Scalar Scalar;
	typedef typename MatrixType::Index Index;
	typedef Matrix<Scalar,Dynamic,1> Vector;
	typedef Matrix<int, 1, MatrixType::ColsAtCompileTime> IntRowVectorType;
	typedef Matrix<int, MatrixType::RowsAtCompileTime, 1> IntColVectorType;
	typedef SparseMatrix<Scalar> LUMatrixType;
	typedef SparseMatrix<Scalar, ColMajor, int> KLUMatrixType;

	KLU() { init(); }
	KLU(const MatrixType& matrix)
	{
		init();
		compute(matrix);
	}

	~KLU()
	{
		if(m_symbolic)
			klu_free_symbolic(&m_symbolic, &m_common);
		if(m_numeric)
			klu_free_numeric(&m_numeric, &m_common);
	}

	inline ComputationInfo info() const
	{ return m_info; }

	void analyzePattern(const MatrixType& matrix)
	{
		if(m_symbolic)
			klu_free_symbolic(&m_symbolic, &m_common);

		grab(matrix);

		m_symbolic = klu_analyze(m_copyMatrix.rows(),
			const_cast<int*>(m_outerIndexPtr), // klu.h says "not modified"
			const_cast<int*>(m_innerIndexPtr),
			&m_common);
		m_info = m_symbolic ? Success : InvalidInput;

		if(!m_symbolic)
			std::cerr << "Symbolic analyze step failed\n";
	}

	void factorize(const MatrixType& matrix)
	{
		eigen_assert(m_symbolic && "KLU: You must first call analyzePattern()");

		if(m_numeric)
			klu_free_numeric(&m_numeric, &m_common);

		grab(matrix);

		m_numeric = klu_factor(
			const_cast<int*>(m_outerIndexPtr),
			const_cast<int*>(m_innerIndexPtr),
			const_cast<double*>(m_valuePtr),
			m_symbolic, &m_common);
		m_info = m_numeric ? Success : InvalidInput;

		if(!m_numeric)
			std::cerr << "Numeric factorization failed\n";
	}

	void refactorize(const MatrixType& matrix)
	{
		eigen_assert(m_symbolic && "KLU: You must first call analyzePattern()");
		eigen_assert(m_numeric && "KLU: You must first call factorize()");

		grab(matrix);

		int ret = klu_refactor(
			const_cast<int*>(m_outerIndexPtr),
			const_cast<int*>(m_innerIndexPtr),
			const_cast<double*>(m_valuePtr),
			m_symbolic, m_numeric, &m_common);
		m_info = ret ? Success : InvalidInput;
	}

	void compute(const MatrixType& matrix)
	{
		analyzePattern(matrix);
		factorize(matrix);
	}

	template<typename Rhs>
	inline const internal::solve_retval<KLU, Rhs> solve(const MatrixBase<Rhs>& b) const
	{
		eigen_assert(m_symbolic && m_numeric && "KLU is not initialized");
		eigen_assert(b.rows() == rows() && "Invalid number of rows on RHS");
		return internal::solve_retval<KLU, Rhs>(*this, b.derived());
	}

	inline Index rows() const { return m_copyMatrix.rows(); }
	inline Index cols() const { return m_copyMatrix.cols(); }

#ifndef EIGEN_PARSED_BY_DOXYGEN
	/** \internal */
	template<typename BDerived, typename XDerived>
	bool _solve(const MatrixBase<BDerived>& b, MatrixBase<XDerived>& x) const
	{
		const int rhsCols = b.cols();
		eigen_assert((BDerived::Flags&RowMajorBit)==0 && "KLU does not support non col-major rhs yet");
		eigen_assert((XDerived::Flags&RowMajorBit)==0 && "KLU does not support non col-major result yet");
		eigen_assert(b.derived().data() != x.derived().data() && "KLU does not support in-place solving");

		int ok;
		for(int j = 0; j < rhsCols; ++j)
		{
			x.col(j) = b.col(j);

			if(m_transpose)
			{
				std::cout << "Solving transpose...\n";
				ok = klu_tsolve(m_symbolic, m_numeric, rows(), rhsCols, &x.const_cast_derived().col(j).coeffRef(0), &m_common);
			}
			else
				ok = klu_solve(m_symbolic, m_numeric, rows(), rhsCols, &x.const_cast_derived().col(j).coeffRef(0), &m_common);
			if(!ok)
				return false;
		}

		return true;
	}
#endif
private:
	mutable klu_common m_common;
	klu_symbolic* m_symbolic = 0;
	klu_numeric* m_numeric = 0;

	KLUMatrixType m_copyMatrix;
	const int* m_outerIndexPtr;
	const int* m_innerIndexPtr;
	const double* m_valuePtr;
	bool m_transpose;

	ComputationInfo m_info;

	void init()
	{
		klu_defaults(&m_common);
	}

	void grab(const MatrixType& matrix)
	{
		if(std::is_same<Index, int>::value)
		{
			m_copyMatrix.resize(matrix.rows(), matrix.cols());
			m_outerIndexPtr = matrix.outerIndexPtr();
			m_innerIndexPtr = matrix.innerIndexPtr();
			m_valuePtr = matrix.valuePtr();

			m_transpose = !!(MatrixType::Flags & RowMajorBit);
		}
		else
		{
			std::cerr << "Warning: costly matrix copy in Eigen::KLU\n";
			m_copyMatrix = matrix; // costly!
			m_outerIndexPtr = m_copyMatrix.outerIndexPtr();
			m_innerIndexPtr = m_copyMatrix.innerIndexPtr();
			m_valuePtr = m_copyMatrix.valuePtr();
		}
	}
};

namespace internal
{

	template<typename _MatrixType, typename Rhs>
	struct solve_retval<KLU<_MatrixType>, Rhs>
	 : solve_retval_base<KLU<_MatrixType>, Rhs>
	{
		typedef KLU<_MatrixType> Dec;
		EIGEN_MAKE_SOLVE_HELPERS(Dec, Rhs)

		template<typename Dest> void evalTo(Dest& dst) const
		{
			dec()._solve(rhs(),dst);
		}
	};

}

}

#endif
