// Fast edge-preserving recursive filter using domain transform
// based on: Domain Transform for Edge-Aware Image and Video Processing
// by Gastal et al., 2011.
// Homepage: http://www.inf.ufrgs.br/~eslgastal/DomainTransform/

#undef HAVE_PCL
#include <depth_filler/domain_transform.h>

#include <opencv/cv.h>

#ifdef CUSTOM_EIGEN

// move this Eigen into a custom namespace (otherwise we get link-time problems)
namespace myEigen
{
#  include CUSTOM_EIGEN
}
namespace Eigen = myEigen::Eigen;

#else
#  include <Eigen/Core>
#endif

#include <chrono>
#include <iostream>

#include <stdarg.h>

#include <immintrin.h>

namespace depth_filler
{

namespace
{
	typedef float Scalar;
	typedef Eigen::Array<Scalar, Eigen::Dynamic, Eigen::Dynamic> Array;
	typedef Eigen::Array<Scalar, Eigen::Dynamic, 1> ArrayCol;

	typedef Eigen::Map<Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> CvMap;

	constexpr bool DEBUG = false;

	void debug(const char* fmt, ...) __attribute__(( format (printf, 1, 2)));
	inline void debug(const char* fmt, ...)
	{
		if(DEBUG)
		{
			va_list list;
			va_start(list, fmt);

			printf("DepthFiller: ");
			vprintf(fmt, list);

			va_end(list);
		}
	}
}

class DomainTransformFillerPrivate
{
public:
	int rows = 0;
	int cols = 0;

	Array grayscale;
	Array F;
	Array FT;
	Array N;
	Array NT;
	Array dHdx;
	Array dVdy;

	Scalar sigma_s = 100;
	Scalar sigma_r = 0.02;
	int numIterations = 2;
};

static inline void recursiveFilterHorizontal(Array& F1, Array& F2, const Array& V)
{
#if defined(CUSTOM_EIGEN) && __AVX__
	int pixel = (F1.cols() - 1) * F1.rows();

	{
		float* f10 = &F1(0,0);
		float* f11 = &F1(0,1);
		float* f20 = &F2(0,0);
		float* f21 = &F2(0,1);
		const float* vin = &V(0,1);

		for(int i = 0; i < pixel; i += 8)
		{
			__m256 v = _mm256_load_ps(vin);

			__m256 val0 = _mm256_load_ps(f10);
			__m256 val1 = _mm256_load_ps(f11);
			val0 = _mm256_sub_ps(val0, val1);
			val0 = _mm256_mul_ps(v, val0);
			val1 = _mm256_add_ps(val1, val0);
			_mm256_store_ps(f11, val1);

			val0 = _mm256_load_ps(f20);
			val1 = _mm256_load_ps(f21);
			val0 = _mm256_sub_ps(val0, val1);
			val0 = _mm256_mul_ps(v, val0);
			val1 = _mm256_add_ps(val1, val0);
			_mm256_store_ps(f21, val1);

			f10 += 8;
			f11 += 8;
			f20 += 8;
			f21 += 8;
			vin += 8;
		}
	}

	// Right->Left filter
	{
		float* f10 = &F1(F1.rows()-8,F1.cols()-2);
		float* f11 = &F1(F1.rows()-8,F1.cols()-1);
		float* f20 = &F2(F1.rows()-8,F1.cols()-2);
		float* f21 = &F2(F1.rows()-8,F1.cols()-1);
		const float* vin = &V(0,F1.cols()-1);

		for(int i = 0; i < pixel; i += 8)
		{
			__m256 v = _mm256_load_ps(vin);

			__m256 val0 = _mm256_load_ps(f10);
			__m256 val1 = _mm256_load_ps(f11);
			val1 = _mm256_sub_ps(val1, val0);
			val1 = _mm256_mul_ps(v, val1);
			val1 = _mm256_add_ps(val0, val1);
			_mm256_store_ps(f10, val1);

			val0 = _mm256_load_ps(f20);
			val1 = _mm256_load_ps(f21);
			val1 = _mm256_sub_ps(val1, val0);
			val1 = _mm256_mul_ps(v, val1);
			val1 = _mm256_add_ps(val0, val1);
			_mm256_store_ps(f20, val1);

			f10 -= 8;
			f11 -= 8;
			f20 -= 8;
			f21 -= 8;
			vin -= 8;
		}
	}
#else
#if __AVX__
#warning You can get better performance by installing Eigen 3.3 and setting CUSTOM_EIGEN
#endif

	for(int x = 1; x < F1.cols(); ++x)
	{
		F1.col(x) += V.col(x) * (F1.col(x-1) - F1.col(x));
		F2.col(x) += V.col(x) * (F2.col(x-1) - F2.col(x));
	}

	for(int x = F1.cols()-2; x >= 0; --x)
	{
		F1.col(x) += V.col(x+1) * (F1.col(x+1) - F1.col(x));
		F2.col(x) += V.col(x+1) * (F2.col(x+1) - F2.col(x));
	}
#endif
}

// Transpose code from http://stackoverflow.com/questions/16737298/what-is-the-fastest-way-to-transpose-a-matrix-in-c
static inline void transpose4x4_SSE(float *A, float *B, const int lda, const int ldb) {
	__m128 row1 = _mm_load_ps(&A[0*lda]);
	__m128 row2 = _mm_load_ps(&A[1*lda]);
	__m128 row3 = _mm_load_ps(&A[2*lda]);
	__m128 row4 = _mm_load_ps(&A[3*lda]);
	_MM_TRANSPOSE4_PS(row1, row2, row3, row4);
	_mm_store_ps(&B[0*ldb], row1);
	_mm_store_ps(&B[1*ldb], row2);
	_mm_store_ps(&B[2*ldb], row3);
	_mm_store_ps(&B[3*ldb], row4);
}

static void transpose_block_SSE4x4(float *A, float *B, const int n, const int m, const int lda, const int ldb ,const int block_size)
{
	for(int i=0; i<n; i+=block_size)
	{
		for(int j=0; j<m; j+=block_size)
		{
			int max_i2 = std::min(i+block_size, n);
			int max_j2 = std::min(j+block_size, m);
			for(int i2=i; i2<max_i2; i2+=4)
			{
				for(int j2=j; j2<max_j2; j2+=4)
					transpose4x4_SSE(&A[i2*lda +j2], &B[j2*ldb + i2], lda, ldb);
			}
		}
	}
}

DomainTransformFiller::DomainTransformFiller()
 : m_d(new DomainTransformFillerPrivate)
{
}

DomainTransformFiller::~DomainTransformFiller()
{
}

cv::Mat_<float> DomainTransformFiller::fillDepth(const cv::Mat_<float>& depth, const cv::Mat_<float>& cvGrayscale)
{
	int iCols = depth.cols;

#if __AVX__ && defined(CUSTOM_EIGEN)
	debug("Using AVX!\n");

	// Round up the rows to the nearest 32 byte value
	int iRows = ((depth.rows + 31)/32) * 32;
#else
	int iRows = depth.rows;
#endif

	int iPixels = iCols * iRows;

	debug("Input size: %dx%d => internal size: %dx%d\n", depth.rows, depth.cols, iRows, iCols);

	if(iRows != m_d->rows || iCols != m_d->cols)
	{
		debug("Resizing!\n");
		m_d->grayscale.resize(iRows, iCols);
		m_d->F.resize(iRows, iCols);
		m_d->N.resize(iRows, iCols);
		m_d->FT.resize(iCols, iRows);
		m_d->NT.resize(iCols, iRows);
		m_d->dHdx.resize(iRows, iCols);
		m_d->dVdy.resize(iRows, iCols);

		m_d->rows = iRows;
		m_d->cols = iCols;
	}

	m_d->grayscale.topRows(depth.rows) = CvMap((float*)cvGrayscale.data, depth.rows, depth.cols) / 255;
	m_d->grayscale.bottomRows(iRows-depth.rows).setZero();

	m_d->N.setZero();
	m_d->F.topRows(depth.rows) = CvMap((float*)depth.data, depth.rows, depth.cols);
	m_d->F.bottomRows(iRows-depth.rows).setZero();

	for(int i = 0; i < iPixels; ++i)
	{
		if(!std::isnan(m_d->F.data()[i]))
			m_d->N.data()[i] = 1.0f;
		else
			m_d->F.data()[i] = 0.0f;
	}

	auto t0 = std::chrono::high_resolution_clock::now();

	// Step 1: Compute domain transform
	{
		// Estimate horizontal and vertical partial derivatives using finite differences
		Array diff_x(iRows, iCols);
		Array diff_y(iRows, iCols);

		diff_x.col(0).setZero();
		diff_x.rightCols(iCols - 1) = m_d->grayscale.rightCols(iCols - 1) - m_d->grayscale.leftCols(iCols - 1);

		diff_y.row(0).setZero();
		diff_y.bottomRows(iRows - 1) = m_d->grayscale.bottomRows(iRows - 1) - m_d->grayscale.topRows(iRows - 1);

		// Compute l1-norm distance & derivatives
		// transpose dHdy since the vertical pass is performed with transposed image
		m_d->dHdx = (1.0 + m_d->sigma_s/m_d->sigma_r * diff_x.abs());
		m_d->dVdy = (1.0 + m_d->sigma_s/m_d->sigma_r * diff_y.abs()).transpose();
	}

	auto t1 = std::chrono::high_resolution_clock::now();

	// Step 2: Perform filtering
	float sigma_H = m_d->sigma_s;

	std::vector<Array> VdHdx(m_d->numIterations);
	std::vector<Array> VdVdy(m_d->numIterations);

	{
		// Pre-calculate filter coefficients
		for(int i = 0; i < m_d->numIterations; ++i)
		{
			float sigma_H_i = sigma_H * sqrtf(3.0f) * std::pow(2.0f, m_d->numIterations - (i+1)) / sqrtf(std::pow(4,m_d->numIterations)-1);
			VdVdy[i] = (-sqrtf(2.0f)/sigma_H_i * m_d->dVdy).exp();
			VdHdx[i] = (-sqrtf(2.0f)/sigma_H_i * m_d->dHdx).exp();
		}

		// Do the filtering iterations
		{
			for(int i = 0; i < m_d->numIterations; ++i)
			{
				recursiveFilterHorizontal(m_d->F, m_d->N, VdHdx[i]);
				transpose_block_SSE4x4(m_d->F.data(), m_d->FT.data(), m_d->F.cols(), m_d->F.rows(), m_d->F.rows(), m_d->F.cols(), 16);
				transpose_block_SSE4x4(m_d->N.data(), m_d->NT.data(), m_d->F.cols(), m_d->F.rows(), m_d->F.rows(), m_d->F.cols(), 16);

				recursiveFilterHorizontal(m_d->FT, m_d->NT, VdVdy[i]);

				transpose_block_SSE4x4(m_d->FT.data(), m_d->F.data(), m_d->F.rows(), m_d->F.cols(), m_d->F.cols(), m_d->F.rows(), 16);
				transpose_block_SSE4x4(m_d->NT.data(), m_d->N.data(), m_d->F.rows(), m_d->F.cols(), m_d->F.cols(), m_d->F.rows(), 16);
			}
		}
	}

	auto t2 = std::chrono::high_resolution_clock::now();
	if(DEBUG)
	{
		std::cout << "Domain transform: " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << " ms\n";
		std::cout << "Filtering: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms\n";
	}

	// Re-normalize
	m_d->F /= m_d->N;

	cv::Mat_<float> ret(depth.rows, depth.cols);
	CvMap mappedRet((float*)ret.data, depth.rows, depth.cols);
	mappedRet = m_d->F.topRows(depth.rows);

	return ret;
}

void DomainTransformFiller::setNumIterations(int iter)
{
	m_d->numIterations = iter;
}

void DomainTransformFiller::setSigmaR(double val)
{
	m_d->sigma_r = val;
}

void DomainTransformFiller::setSigmaS(double val)
{
	m_d->sigma_s = val;
}

}
