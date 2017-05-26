// Encapsulates liblinear SVM training
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <Eigen/Core>

#include "sample_db.h"

namespace apc_densecap
{

class SVMModel
{
public:
	bool train(
		const SampleDB& samples, const std::vector<std::string>& presentItems,
		const std::string& target, const Parameters& params,
		const boost::filesystem::path& exclude = boost::filesystem::path()
	);

	float score(const Eigen::VectorXf& features);

	template<class Derived>
	Eigen::VectorXf scoreBatch(const Eigen::MatrixBase<Derived>& features);
private:
	Eigen::VectorXf m_weights;
	float m_biasScale;
};

template<class Derived>
Eigen::VectorXf SVMModel::scoreBatch(const Eigen::MatrixBase<Derived>& features)
{
	return features * m_weights.head(features.cols())
		+ Eigen::VectorXf::Constant(features.rows(), m_biasScale * m_weights.tail(1)[0]);
}

}
