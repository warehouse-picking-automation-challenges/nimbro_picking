// Depth filling based on the paper
//  "Colorization Using Optimization" by A. Levin, D. Lischinski, Y. Weiss
// Author: Max Schwarz <max.schwarz@uni-bonn.de>
//  based on an implementation by Benedikt Waldvogel

#include <depth_filler/depth_filler.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/LU>

#ifdef EIGEN_UMFPACK_SUPPORT
#include <Eigen/UmfPackSupport>
#endif

#ifdef KLU_SUPPORT
#include "eigen_klu.h"
#endif

#include <unsupported/Eigen/SparseExtra>

#include <chrono>

namespace depth_filler
{

constexpr bool DUMP_MATRICES = false;
constexpr bool PROFILE_SOLVING = true;

typedef decltype(std::chrono::high_resolution_clock::now()) Time;

// Choose best available solver
#ifdef KLU_SUPPORT
// KLU can only solve in double precision
typedef double SolveType;
#elif defined(EIGEN_UMFPACK_SUPPORT)
#warning Using slower UMFPACK solver. Install KLU for faster solving.
// UMFPACK can only solve in double precision
typedef double SolveType;
#else
#warning Using very slow built-in Eigen BiCGSTAB solver. Install KLU!
typedef float SolveType;
#endif

namespace
{
	class ScopeTimer
	{
	public:
		explicit ScopeTimer(const std::string& name)
		: m_name(name)
		{
			if(PROFILE_SOLVING)
				m_start = std::chrono::high_resolution_clock::now();
		}

		~ScopeTimer()
		{
			if(PROFILE_SOLVING)
			{
				Time stop = std::chrono::high_resolution_clock::now();
				std::cout << m_name << " took " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - m_start).count() << "ms\n";
			}
		}
	private:
		std::string m_name;
		Time m_start;
	};
}

class DepthFillerPrivate
{
public:
	void solve();

	int rows;
	int cols;

	cv::Mat_<uint8_t> mask;
	cv::Mat_<float> depth;
	cv::Mat_<cv::Vec3b> rgb;
	unsigned int numUnknown;

	cv::Mat_<int> mapImgToIdx;

	std::vector<int> outerIndices;
	std::vector<int> innerIndices;
	std::vector<SolveType> values;

	Eigen::Matrix<SolveType, Eigen::Dynamic, 1> b;
	Eigen::Matrix<SolveType, Eigen::Dynamic, 1> newDepth;

#if defined(KLU_SUPPORT)
	Eigen::KLU<Eigen::MappedSparseMatrix<SolveType, Eigen::RowMajor>> solver;
#elif defined(EIGEN_UMFPACK_SUPPORT)
	Eigen::UmfPackLU<Eigen::SparseMatrix<SolveType>> solver;
#else
	Eigen::BiCGSTAB<Eigen::MappedSparseMatrix<SolveType, Eigen::RowMajor>> solver;
#endif

	DepthFiller::ColorDistance distance = DepthFiller::CD_GRAYSCALE;
	bool normalizeWithVariance = true;
	float distExponent = 1.2f;
	float distScale = 1.0f;
};


void DepthFillerPrivate::solve()
{
	const int winRad = 1;
	const int winPixel = (2 * winRad + 1)*(2 * winRad + 1);

	outerIndices.clear();
	innerIndices.clear();
	values.clear();

	const int GUESS_NONZERO = 9 * numUnknown;
	outerIndices.reserve(numUnknown);
	innerIndices.reserve(GUESS_NONZERO);
	values.reserve(GUESS_NONZERO);

	Eigen::VectorXi cols(winPixel);
	Eigen::VectorXf winDepth(winPixel);
	Eigen::VectorXf gvals = Eigen::VectorXf::Zero(winPixel);
	std::vector<Eigen::Vector3f> rgbWin(winPixel);

	b.resize(numUnknown);
	b.setZero();

	mapImgToIdx = cv::Mat_<int>(depth.rows, depth.cols);
	{
		unsigned int idx = 0;
		for(int y = 0; y < depth.rows; ++y)
		{
			float* ptr = depth[y];
			int* idxPtr = mapImgToIdx[y];

			for(int x = 0; x < depth.cols; ++x)
			{
				if(ptr[x] < 0 && (!mask.rows || mask(y,x)))
					idxPtr[x] = idx++;
				else
					idxPtr[x] = -1;
			}
		}
	}

	{
		ScopeTimer timer("formulate system matrix");

		cv::Mat_<uint8_t> gray;
		cv::cvtColor(rgb, gray, CV_BGR2GRAY);

		int absIdx = 0;

		for(int y = 0; y < depth.rows; ++y)
		{
			for(int x = 0; x < depth.cols; ++x)
			{
				// For pixels r without depth, we need to put the equation
				//  U(r) - sum(w_rs * U(s)) = 0
				// into matrix form (sum over neighborhood of r).
				// For pixels r with depth, we have
				//  U(r) = depth.

				if(depth(y,x) >= 0 || (mask.rows && !mask(y,x)))
					continue; // this pixel is known

				outerIndices.push_back(innerIndices.size());

				int nWin = 0;

				for(int wy = std::max(0, y - winRad); wy < std::min(y + winRad + 1, depth.rows); ++wy)
				{
					for(int wx = std::max(0, x - winRad); wx < std::min(x + winRad + 1, depth.cols); ++wx)
					{
#if REDUCED_NEIGHBORHOOD
						if(std::abs(wx-x) == 1 && std::abs(wy-y) == 1)
							continue;
#endif

						if(wx == x && wy == y)
							continue;

						cols[nWin] = mapImgToIdx(wy, wx);
						gvals[nWin] = gray(wy, wx);
						winDepth[nWin] = depth(wy,wx);

						auto& color = rgb.at<cv::Vec3b>(y,x);
						rgbWin[nWin] = Eigen::Vector3f((float)color[0] / 255, (float)color[1]/255, (float)color[2]/255);

						nWin++;
					}
				}

				// center pixel
				float curVal = gray(y, x);
				gvals[nWin] = curVal;

				auto& color = rgb.at<cv::Vec3b>(y,x);
				auto ownColor = Eigen::Vector3f((float)color[0] / 255, (float)color[1]/255, (float)color[2]/255);

				// Calculate variance
				float mean = gvals.mean();
				Eigen::VectorXf dev = gvals - Eigen::VectorXf::Constant(gvals.rows(), mean);
				float c_var = (dev.array()*dev.array()).mean();

				float csig = normalizeWithVariance ? std::max(c_var, 0.0000002f) : 1.0f;

				Eigen::ArrayXf tmp(nWin);
				if(distance == DepthFiller::CD_GRAYSCALE)
				{
					tmp = gvals.head(nWin).array() - curVal;
					tmp = (tmp * tmp).pow(distExponent);
				}
				else if(distance == DepthFiller::CD_RGB)
				{
					// RGB distance
					for(int i = 0; i < nWin; ++i)
					{
						tmp[i] = std::pow((ownColor - rgbWin[i]).squaredNorm(), distExponent);
					}
				}
				tmp *= distScale;

				gvals.head(nWin) = (- tmp / csig).exp().matrix();

				float s = gvals.head(nWin).sum();
				if(s > 0)
				{
					gvals.head(nWin) /= s;
				}

				bool self = false;
				for(int i = 0; i < nWin; ++i)
				{
					if(!self && cols[i] > absIdx)
					{
						// Now the self-reference U(r) (along the diagonal).
						innerIndices.push_back(absIdx);
						values.push_back(1.0);
						self = true;
					}

					if(cols[i] >= 0)
					{
						// relation to other unknown pixel...
						innerIndices.push_back(cols[i]);
						values.push_back(-gvals[i]);
					}
					else
					{
						// relation to fixed pixel => put on rhs
						b[absIdx] += gvals[i] * winDepth[i];
					}
				}
				if(!self)
				{
					innerIndices.push_back(absIdx);
					values.push_back(1.0);
				}

				absIdx++;
			}
		}
	}

	outerIndices.push_back(innerIndices.size());

	// Create Eigen matrix from A
	Eigen::MappedSparseMatrix<SolveType, Eigen::RowMajor> A(
		numUnknown, numUnknown,
		values.size(),
		outerIndices.data(), innerIndices.data(),
		values.data()
	);
	{
		ScopeTimer timer("sparse matrix");

		// Check CSR format
		if(outerIndices.size() != numUnknown+1)
			throw std::runtime_error("outer indices has invalid size");

		printf("numUnknown: %u, non-zero: %lu\n", numUnknown, values.size());

		for(unsigned int i = 0; i < numUnknown; ++i)
		{
			int rowStart = outerIndices[i];
			int nextRowStart = outerIndices[i+1];

			if(rowStart > nextRowStart)
				throw std::runtime_error("outer indices are non-monotonic");

			if(rowStart >= (int)values.size() || rowStart < 0)
				throw std::runtime_error("invalid start idx");

			if(nextRowStart > (int)values.size() || nextRowStart < 0)
				throw std::runtime_error("invalid end idx");

			int lastOne = -1;
			for(int j = rowStart; j < nextRowStart; ++j)
			{
				int col = innerIndices[j];
				if(col < 0 || col >= (int)numUnknown)
					throw std::runtime_error("invalid inner index");

				if(col <= lastOne)
					throw std::runtime_error("non-monotonic inner index");

				lastOne = col;
			}
		}

		if(DUMP_MATRICES)
		{
			Eigen::saveMarket(A, "/tmp/filler_A.mtx");
			Eigen::saveMarketVector(b, "/tmp/filler_A_b.mtx");

			// Dump upper left corner of matrix as image
			const int IMG_SIZE = std::min<int>(5000, A.rows());
			cv::Mat_<uint8_t> vis(IMG_SIZE, IMG_SIZE, 255);
			Eigen::MatrixXf reduced(IMG_SIZE, IMG_SIZE);
			reduced.fill(0);

			for (int k=0; k<A.outerSize(); ++k)
			{
				for (Eigen::MappedSparseMatrix<SolveType, Eigen::RowMajor>::InnerIterator it(A,k); it; ++it)
				{
					if(it.row() < 0 || it.col() < 0)
					{
						fprintf(stderr, "Invalid idx!\n");
					}
					if(it.row() < IMG_SIZE && it.col() < IMG_SIZE)
					{
						vis(it.row(), it.col()) = 0;
						reduced(it.row(), it.col()) = it.value();
					}
				}
			}

			cv::imwrite("/tmp/filler_A.png", vis);

			Eigen::MatrixXf inverse = reduced.inverse();
			cv::Mat_<uint8_t> inverseVis(IMG_SIZE, IMG_SIZE);
			for(int i = 0; i < IMG_SIZE; ++i)
			{
				for(int j = 0; j < IMG_SIZE; ++j)
				{
					inverseVis(i,j) = (std::abs(inverse(i,j)) > 1e-6) ? 0 : 255;
				}
			}

			cv::imwrite("/tmp/filler_Ainv.png", inverseVis);
		}
	}

	{
		ScopeTimer timer("solving");

#if KLU_SUPPORT
// 		if(sameStructure)
// 			solver.refactorize(A);
// 		else
			solver.compute(A);
#else
		solver.compute(Eigen::SparseMatrix<SolveType>(A));
#endif
		if(solver.info() != Eigen::Success)
		{
			fprintf(stderr, "Failed to solve system!\n");
		}
		newDepth = solver.solve(b);
	}
}



DepthFiller::DepthFiller()
 : m_d(new DepthFillerPrivate)
{
}

DepthFiller::~DepthFiller()
{
}

void DepthFiller::setColorDistance(ColorDistance dist)
{
	m_d->distance = dist;
}

void DepthFiller::setNormalizeWithVariance(bool on)
{
	m_d->normalizeWithVariance = on;
}

void DepthFiller::setDistanceExponent(float exp)
{
	m_d->distExponent = exp;
}

void DepthFiller::setDistanceScale(float scale)
{
	m_d->distScale = scale;
}

cv::Mat DepthFiller::fillDepth(const cv::Mat& input, const cv::Mat& rgb, bool sameStructure, const cv::Mat_<uint8_t>& mask)
{
	m_d->rows = input.rows;
	m_d->cols = input.cols;

	// Normalize depth image to [0, 1]
	m_d->depth = cv::Mat_<float>(input.size());
	float oldMin = std::numeric_limits<float>::infinity();
	float oldMax = -std::numeric_limits<float>::infinity();

	m_d->numUnknown = 0;

	{
		ScopeTimer timer("normalize depth image");

		for(int y = 0; y < input.rows; ++y)
		{
			for(int x = 0; x < input.cols; ++x)
			{
				float ival = input.at<float>(y,x);
				if(!std::isfinite(ival) || ival == 0)
					continue;

				oldMin = std::min<float>(oldMin, ival);
				oldMax = std::max<float>(oldMax, ival);
			}
		}

		for(int y = 0; y < input.rows; ++y)
		{
			for(int x = 0; x < input.cols; ++x)
			{
				float ival = input.at<float>(y,x);
				if(!std::isfinite(ival) || ival == 0)
				{
					m_d->depth(y,x) = -1;

					if(!mask.rows || mask(y,x))
						m_d->numUnknown++;
				}
				else
					m_d->depth(y,x) = ((float)(ival - oldMin)) / (oldMax - oldMin);
			}
		}
	}
	printf("depth: %d by %d, numUnknown: %u\n", m_d->depth.cols, m_d->depth.rows, m_d->numUnknown);

	m_d->rgb = rgb;
	m_d->mask = mask;

	m_d->solve();

	cv::Mat_<float> ret(input.size());
	int idx = 0;
	for(int y = 0; y < ret.rows; ++y)
	{
		for(int x = 0; x < ret.cols; ++x)
		{
			float ival = input.at<float>(y,x);
			if(std::isfinite(ival) || (mask.rows && !mask(y,x)))
				ret(y,x) = ival;
			else
			{
				ret(y,x) = oldMin + (oldMax - oldMin) * m_d->newDepth[idx];
				idx++;
			}
		}
	}

	return ret;
}

void DepthFiller::erodeDepth(const cv::Mat_<float>& input, cv::Mat_<float>& output, int kernelSize)
{
	cv::Mat_<uint8_t> valid(input.rows, input.cols);
	for(int y = 0; y < input.rows; ++y)
	{
		for(int x = 0; x < input.cols; ++x)
		{
			valid(y,x) = std::isfinite(input(y,x));
		}
	}

	cv::Mat_<uint8_t> erodedValid;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*kernelSize+1, 2*kernelSize+1), cv::Point(kernelSize, kernelSize));
	cv::erode(valid, erodedValid, kernel);

	for(int y = 0; y < valid.rows; ++y)
	{
		for(int x = 0; x < valid.cols; ++x)
		{
			if(!erodedValid(y,x))
				output(y,x) = NAN;
		}
	}
}

cv::Mat_<float> DepthFiller::prefill(const cv::Mat_<float>& input)
{
	cv::Mat_<float> ret;
	input.copyTo(ret);

	Eigen::Matrix<float, 9, 1> values;
	cv::Mat_<bool> valid(3,3);

	for(int y = 1; y < input.rows-1; ++y)
	{
		for(int x = 1; x < input.cols-1; ++x)
		{
			if(std::isfinite(input(y,x)))
			{
				ret(y,x) = input(y,x);
				continue;
			}

			valid = false;

			int numValid = 0;

			for(int dy = -1; dy <= 1; ++dy)
			{
				for(int dx = -1; dx <= 1; ++dx)
				{
					if(std::isfinite(input(y+dy,x+dx)))
					{
						valid[dy+1][dx+1] = true;

						values[numValid] = input(y+dy,x+dx);
						numValid++;
					}
				}
			}

			bool corners[4] = {
				valid[0][0] || valid[0][1] || valid[1][0],
				valid[2][0] || valid[1][0] || valid[2][1],
				valid[0][2] || valid[1][2] || valid[0][1],
				valid[2][2] || valid[1][2] || valid[2][1]
			};

			if(corners[0] && corners[1] && corners[2] && corners[3]
				&& values.head(numValid).maxCoeff() - values.head(numValid).minCoeff() < 0.05)
			{
				ret(y,x) = values.head(numValid).mean();
			}
			else
				ret(y,x) = NAN;
		}
	}

	return ret;
}

void DepthFiller::dumpSystem(const std::string& prefix)
{
	Eigen::MappedSparseMatrix<SolveType, Eigen::RowMajor> A(
		m_d->numUnknown, m_d->numUnknown,
		m_d->values.size(),
		m_d->outerIndices.data(), m_d->innerIndices.data(),
		m_d->values.data()
	);

	Eigen::saveMarket(A, prefix + ".mtx");
	Eigen::saveMarketVector(m_d->b, prefix + "_b.mtx");
}

}
