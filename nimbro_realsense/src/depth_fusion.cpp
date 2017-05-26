// Nodelet for fusing (i.e. projecting together) two depth streams
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "depth_fusion.h"

#include <pcl_ros/transforms.h>

#include <pluginlib/class_list_macros.h>

#include <opencv/highgui.h>

#include <chrono>

namespace
{
	typedef std::chrono::high_resolution_clock clock;

	inline int milliseconds(clock::duration duration)
	{ return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count(); }

	constexpr int SUBSAMPLE = 1;
}

namespace
{

cv::Mat_<float> prefill(const cv::Mat_<float>& input)
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

}

namespace nimbro_realsense
{

DepthFusion::DepthFusion()
 : m_param_fill_input("fill_input", true)
 , m_param_weight_upper("weights/upper", 0.0f, 0.01f, 1.0f, 1.0f)
 , m_param_weight_lower("weights/lower", 0.0f, 0.01f, 1.0f, 0.4f)
 , m_param_weight_stereo("weights/stereo", 0.0f, 0.01f, 1.0f, 0.05f)
 , m_param_trust_stereo_threshold("trust_stereo", 0.0f, 0.001f, 0.2f, 0.06f)
{
}

DepthFusion::~DepthFusion()
{
	m_shouldExit = true;
	m_cond.notify_all();
	m_processThread.join();
}

void DepthFusion::onInit()
{
	auto& nh = getPrivateNodeHandle();

	std::string extrinsicsFile;
	if(!nh.getParam("extrinsics", extrinsicsFile))
	{
		NODELET_ERROR("need extrinsics parameter");
		throw std::runtime_error("need ~extrinsics parameter");
	}

	{
		std::ifstream stream(extrinsicsFile);

		std::string line;

		bool t = false;
		int idx = 0;

		while(true)
		{
			std::getline(stream, line);
			if(stream.bad() || stream.eof())
				break;

			if(line[0] == '#')
				continue;

			std::stringstream ss(line);
			if(!t)
			{
				ss >> m_R(idx,0) >> m_R(idx,1) >> m_R(idx,2);
				idx++;
				if(idx == 3)
				{
					t = true;
					idx = 0;
				}
			}
			else
			{
				ss >> m_T[idx];
				idx++;
				if(idx == 3)
					break;
			}
		}

		NODELET_INFO("rotated extrinsics:");
		NODELET_INFO_STREAM(m_R);
		NODELET_INFO_STREAM("T: " << m_T.transpose());

		// Extrinsics are calibrated on the stereo images, which are rotated.
		// undo the rotation here:
		Eigen::Matrix3f rot1, rot2;
		rot1 <<  0.0f, -1.0f, 0.0f,
		         1.0f,  0.0f, 0.0f,
		         0.0f,  0.0f, 1.0f;
		rot2 <<  0.0f, -1.0f, 0.0f,
		         1.0f,  0.0f, 0.0f,
		         0.0f,  0.0f, 1.0f;

		m_R = rot2 * m_R * rot1;
		m_T = rot2 * m_T;

		NODELET_INFO("unrotated extrinsics:");
		NODELET_INFO_STREAM(m_R);
		NODELET_INFO_STREAM("T: " << m_T.transpose());
	}

	m_sub_camInfo_upper = nh.subscribe("upper/info", 1, &DepthFusion::handleCameraInfoUpper, this);
	m_sub_camInfo_lower = nh.subscribe("lower/info", 1, &DepthFusion::handleCameraInfoLower, this);
	m_sub_camInfo_stereo = nh.subscribe("stereo/info", 1, &DepthFusion::handleCameraInfoStereo, this);

	auto cb = boost::bind(&DepthFusion::checkSubscribers, this);
	m_pub_cloud = nh.advertise<Cloud>("output", 1, cb, cb);
	m_pub_correctedStereo = nh.advertise<Cloud>("correctedStereo", 1);

	m_sub_cloud_upper.registerCallback(boost::bind(&DepthFusion::debugCloud, this, "upper", _1));
	m_sub_cloud_lower.registerCallback(boost::bind(&DepthFusion::debugCloud, this, "lower", _1));
	m_sub_cloud_stereoInUpper.registerCallback(boost::bind(&DepthFusion::debugCloud, this, "stereo", _1));

	m_sync_cloud.reset(new CloudSynchronizer(
		CloudSyncPolicy(10),
		m_sub_cloud_upper, m_sub_cloud_lower, m_sub_cloud_stereoInUpper
	));
	m_sync_cloud->registerCallback(boost::bind(&DepthFusion::handleClouds, this, _1, _2, _3));

	checkSubscribers();

	m_processThread = std::thread(std::bind(&DepthFusion::processThread, this));
}

void DepthFusion::debugCloud(const std::string& name, const sensor_msgs::PointCloud2ConstPtr& msg)
{
	NODELET_DEBUG("INPUT: %20s (%lu)", name.c_str(), msg->header.stamp.toNSec());
}

void DepthFusion::checkSubscribers()
{
	bool needed = m_pub_cloud.getNumSubscribers() != 0;

	if(needed && !m_active)
	{
		NODELET_INFO("Got a subscriber, starting");
		ros::NodeHandle nh = getPrivateNodeHandle();

		// Discard old data
		m_cloud_upper.reset();
		m_cloud_lower.reset();
		m_cloud_stereo.reset();

		m_subscribeTime = ros::Time::now();

		m_sub_cloud_upper.subscribe(nh, "upper/cloud", 1, ros::TransportHints().tcpNoDelay());
		m_sub_cloud_lower.subscribe(nh, "lower/cloud", 1, ros::TransportHints().tcpNoDelay());
		m_sub_cloud_stereoInUpper.subscribe(nh, "stereo/cloud", 1, ros::TransportHints().tcpNoDelay());

		m_active = true;
	}
	else if(!needed && m_active)
	{
		NODELET_INFO("No subscribers left, stopping");

		m_sub_cloud_upper.unsubscribe();
		m_sub_cloud_lower.unsubscribe();
		m_sub_cloud_stereoInUpper.unsubscribe();

		m_active = false;
	}
}

void DepthFusion::handleCameraInfoUpper(const sensor_msgs::CameraInfoConstPtr& info)
{
	m_model_upper.fromCameraInfo(info);
	m_sub_camInfo_upper.shutdown();
}

void DepthFusion::handleCameraInfoLower(const sensor_msgs::CameraInfoConstPtr& info)
{
	m_model_lower.fromCameraInfo(info);
	m_sub_camInfo_lower.shutdown();
}

void DepthFusion::handleCameraInfoStereo(const sensor_msgs::CameraInfoConstPtr& info)
{
	m_model_stereo.fromCameraInfo(info);
	m_sub_camInfo_stereo.shutdown();
}

void DepthFusion::handleClouds(
	const sensor_msgs::PointCloud2ConstPtr& upper,
	const sensor_msgs::PointCloud2ConstPtr& lower,
	const sensor_msgs::PointCloud2ConstPtr& stereo
)
{
	if(upper->header.stamp < m_subscribeTime)
		return;

	NODELET_DEBUG("input latency: %5.3fs", (ros::Time::now() - upper->header.stamp).toSec());

	{
		std::lock_guard<std::mutex> lock(m_mutex); m_cloud_upper = upper;
		m_cloud_lower = lower;
		m_cloud_stereo = stereo;
	}
	m_cond.notify_all();
}

void DepthFusion::processThread()
{
	while(!m_shouldExit)
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		m_cond.wait(lock);

		if(m_cloud_upper && m_cloud_lower && m_cloud_stereo &&
			m_model_upper.initialized() && m_model_lower.initialized() && m_model_stereo.initialized())
		{
			sensor_msgs::PointCloud2ConstPtr proc_cloud_upper; proc_cloud_upper.swap(m_cloud_upper);
			sensor_msgs::PointCloud2ConstPtr proc_cloud_lower; proc_cloud_lower.swap(m_cloud_lower);
			sensor_msgs::PointCloud2ConstPtr proc_cloud_stereo; proc_cloud_stereo.swap(m_cloud_stereo);

			lock.unlock();

			Cloud::Ptr upperPCL(new Cloud);
			pcl::fromROSMsg(*proc_cloud_upper, *upperPCL);

			Cloud::Ptr lowerPCL(new Cloud);
			pcl::fromROSMsg(*proc_cloud_lower, *lowerPCL);

			Cloud::Ptr stereoPCL(new Cloud);
			pcl::fromROSMsg(*proc_cloud_stereo, *stereoPCL);

			process(*upperPCL, *lowerPCL, *stereoPCL);
		}
	}
}

void DepthFusion::process(const Cloud& cloud_upper, const Cloud& cloud_lower, const Cloud& cloud_stereo)
{
	NODELET_DEBUG("Fusion started. Time stamps: %lu %lu %lu",
		cloud_upper.header.stamp,
		cloud_lower.header.stamp,
		cloud_stereo.header.stamp
	);

	if(!cloud_upper.isOrganized())
	{
		NODELET_ERROR("upper cloud is not organized");
		return;
	}
	if(!cloud_lower.isOrganized())
	{
		NODELET_ERROR("upper cloud is not organized");
		return;
	}
	if(!cloud_stereo.isOrganized())
	{
		NODELET_ERROR("upper cloud is not organized");
		return;
	}

	// cloud_stereo was generated by a stereo algorithm on the rectified image.
	// unrectify it to get a depth value for each original pixel
	cv::Mat_<float> depth_stereo(cloud_upper.height/SUBSAMPLE, cloud_upper.width/SUBSAMPLE, NAN);

	cv::Mat_<float> depth_upper;
	cv::Mat_<float> depth_lower;
	cv::Mat_<cv::Vec3b> rgbFull(cloud_upper.height, cloud_upper.width);
	{
		auto start = clock::now();

		// Downscale the depth
		cv::Mat_<float> tmp_upper(cloud_upper.height/SUBSAMPLE, cloud_upper.width/SUBSAMPLE);
		tmp_upper = NAN;

		cv::Mat_<float> tmp_lower(cloud_upper.height/SUBSAMPLE, cloud_upper.width/SUBSAMPLE);
		tmp_lower = NAN;

		unsigned int idx = 0;
		for(unsigned int y = 0; y < cloud_upper.height; ++y)
		{
			for(unsigned int x = 0; x < cloud_upper.width; ++x, idx++)
			{
				int dx = x/SUBSAMPLE;
				int dy = y/SUBSAMPLE;

				if(!std::isfinite(tmp_upper(dy,dx)) || cloud_upper[idx].z < tmp_upper(dy,dx))
					tmp_upper(dy,dx) = cloud_upper[idx].z;

				if(std::isfinite(cloud_lower[idx].z))
				{
					const auto& p = cloud_lower[idx];

					Eigen::Vector3f inUpper = m_R.transpose() * (p.getVector3fMap() - m_T);

					cv::Point2d pix = m_model_upper.project3dToPixel(cv::Point3d(inUpper.x(), inUpper.y(), inUpper.z()));
					cv::Point2i rounded(std::round(pix.x/SUBSAMPLE), std::round(pix.y/SUBSAMPLE));

					if(rounded.x >= 0 && rounded.y >= 0 && rounded.x < tmp_lower.cols && rounded.y < tmp_lower.rows)
					{
						if(!std::isfinite(tmp_lower(rounded.y, rounded.x)) || inUpper.z() < tmp_lower(rounded.y, rounded.x))
						{
							tmp_lower(rounded.y, rounded.x) = inUpper.z();
						}
					}
				}

				rgbFull(y,x) = cv::Vec3b(cloud_upper[idx].b, cloud_upper[idx].g, cloud_upper[idx].r);
			}
		}

		depth_upper = prefill(tmp_upper);
		depth_lower = prefill(tmp_lower);

		auto end = clock::now();
		NODELET_DEBUG("RGB-D mapping took %d ms", milliseconds(end - start));
	}

	if(!m_stereoMapsInitialized)
	{
		cv::initUndistortRectifyMap(
			m_model_stereo.intrinsicMatrix(),
			m_model_stereo.distortionCoeffs(),
			m_model_stereo.rotationMatrix(),
			m_model_stereo.projectionMatrix(),
			m_model_stereo.fullResolution(),
			CV_32FC1,
			m_stereoMapX, m_stereoMapY
		);
		NODELET_INFO("stereo resolution: %dx%d", m_model_stereo.fullResolution().height, m_model_stereo.fullResolution().width);
		NODELET_INFO("Initialized maps: %dx%d and %dx%d", m_stereoMapX.rows, m_stereoMapX.cols, m_stereoMapY.rows, m_stereoMapY.cols);
		NODELET_INFO("(0,0) maps to (%f,%f)", m_stereoMapX(0,0), m_stereoMapY(0,0));

		m_stereoMapsInitialized = true;
	}

	{
		auto start = clock::now();

		for(const Point& p : cloud_stereo)
		{
			if(!std::isfinite(p.z))
				continue;

			cv::Point3d cvPoint(p.x, p.y, p.z);
			cv::Point2d cvDistortedPixel = m_model_stereo.project3dToPixel(cvPoint);
			cv::Point2i cvDistortedRounded(std::round(cvDistortedPixel.x), std::round(cvDistortedPixel.y));

			// This is awfully slow!
// 			cv::Point2d cvPixel = m_model_stereo.unrectifyPoint(cvDistortedRounded);
			cv::Point2d cvPixel(
				m_stereoMapX(cvDistortedRounded.y, cvDistortedRounded.x),
				m_stereoMapY(cvDistortedRounded.y, cvDistortedRounded.x)
			);

			// to unrotated original image
			cv::Point2d orig(cvPixel.y, cloud_upper.height - 1 - cvPixel.x);

			cv::Point2i rounded(std::round(orig.x), std::round(orig.y));

			int dx = rounded.x/SUBSAMPLE;
			int dy = rounded.y/SUBSAMPLE;

			if(dx < 0 || dx >= depth_stereo.cols || dy < 0 || dy >= depth_stereo.rows)
				continue;

			depth_stereo(dy, dx) = p.z;
		}

		auto end = clock::now();
		NODELET_DEBUG("Stereo mapping took %d ms", milliseconds(end - start));
	}

	// Downscale RGB
	cv::Mat_<cv::Vec3b> rgb;
	cv::resize(rgbFull, rgb, depth_upper.size(), 0, 0, cv::INTER_AREA);

	if(m_calibrate)
	{
		NODELET_INFO("Calibrating...");

		if(false)
		{
			Eigen::Vector2f result;
			if(!calibrateDepthMaps(depth_stereo, depth_upper, &result))
			{
				NODELET_WARN("stereo calibration failed");
				return;
			}

			m_calibOffset = result[0];
			m_calibScale = result[1];

			if(!calibrateDepthMaps(depth_lower, depth_upper, &result))
			{
				NODELET_WARN("upper/lower calibration failed");
				return;
			}

			m_lowerOffset = result[0];
			m_lowerScale = result[1];
		}
		else
		{
			m_calibOffset = 0.012853268;
			m_calibScale = 0.9648272;

			m_lowerOffset = 0.001032265;
			m_lowerScale = 0.9813212;
		}

		m_calibrate = false;

		NODELET_INFO("Calibration finished");
	}

	// Apply calibration
	depth_lower = m_lowerScale * depth_lower + m_lowerOffset;
	depth_stereo = m_calibScale * depth_stereo + m_calibOffset;

	if(m_param_fill_input())
	{
		cv::Mat_<uint8_t> grayscale;
		cv::cvtColor(rgb, grayscale, cv::COLOR_RGB2GRAY);

		depth_lower = m_filler.fillDepth(depth_lower, grayscale);
		depth_upper = m_filler.fillDepth(depth_upper, grayscale);
		depth_stereo = m_filler.fillDepth(depth_stereo, grayscale);
	}

	Cloud::Ptr output(new Cloud);
	{
		auto start = clock::now();

		output->header = cloud_upper.header;
		output->resize(depth_upper.rows*depth_upper.cols);
		output->width = depth_upper.cols;
		output->height = depth_upper.rows;

		cv::Mat_<float> weight_upper(depth_upper.rows, depth_upper.cols);
		cv::Mat_<float> weight_lower(depth_upper.rows, depth_upper.cols);
		cv::Mat_<float> weight_stereo(depth_upper.rows, depth_upper.cols);

		weight_upper = 0.0f;
		weight_lower = 0.0f;
		weight_stereo = 0.0f;

		std::vector<Measurement> measurements;
		std::vector<Measurement> inliers;
		std::vector<Measurement> currentInliers;

		for(int y = 0; y < depth_upper.rows; ++y)
		{
			for(int x = 0; x < depth_upper.cols; ++x)
			{
				float stereo = depth_stereo(y,x);
				float upper = depth_upper(y,x);
				float lower = depth_lower(y,x);

				measurements.clear();
				if(std::isfinite(stereo))
					measurements.emplace_back(Measurement::SOURCE_STEREO, stereo, m_param_weight_stereo());
				if(std::isfinite(upper))
					measurements.emplace_back(Measurement::SOURCE_UPPER_RGBD, upper, m_param_weight_upper());
				if(std::isfinite(lower))
					measurements.emplace_back(Measurement::SOURCE_LOWER_RGBD, lower, m_param_weight_lower());

				float depth = NAN;
				inliers.clear();
				float weightSum;
				if(measurements.size() != 0)
				{
					unsigned int max_inliers = 0;
					int max_idx = -1;

					for(std::size_t i = 0; i < measurements.size(); ++i)
					{
						// count inliers
						currentInliers.clear();
						unsigned int numInliers = 0;
						float currentMean = 0.0f;
						float currentWeight = 0.0f;

						for(std::size_t j = 0; j < measurements.size(); ++j)
						{
							float diff = std::abs(measurements[i].depth - measurements[j].depth);
							if(diff < 0.03)
							{
								currentMean += measurements[j].weight * measurements[j].depth;
								currentWeight += measurements[j].weight;
								numInliers++;
								currentInliers.push_back(measurements[j]);
							}
						}

						if(numInliers > max_inliers || (measurements[i].source != Measurement::SOURCE_STEREO && numInliers >= max_inliers))
						{
							depth = currentMean / currentWeight;
							max_inliers = numInliers;
							max_idx = i;
							inliers.swap(currentInliers);
							weightSum = currentWeight;
						}
					}

					if(measurements[max_idx].source != Measurement::SOURCE_STEREO && max_inliers < 2)
						depth = NAN;
				}

				// Special case: RGB-D will sometimes measure *through* items (e.g. penholder),
				// so trust stereo if it is significantly smaller.
				if(std::isfinite(stereo) && depth - stereo > m_param_trust_stereo_threshold())
					depth = stereo;

				for(auto& measurement : inliers)
				{
					switch(measurement.source)
					{
						case Measurement::SOURCE_LOWER_RGBD:
							weight_lower(y,x) = measurement.weight / weightSum;
							break;
						case Measurement::SOURCE_UPPER_RGBD:
							weight_upper(y,x) = measurement.weight / weightSum;
							break;
						case Measurement::SOURCE_STEREO:
							weight_stereo(y,x) = measurement.weight / weightSum;
							break;
					}
				}

				Point& p1 = (*output)[y * output->width + x];

#warning Please check if the upscaling (2*) is correct here
				p1.x = depth * (SUBSAMPLE*x - m_model_upper.cx()) / m_model_upper.fx();
				p1.y = depth * (SUBSAMPLE*y - m_model_upper.cy()) / m_model_upper.fy();
				p1.z = depth;

				auto color = rgb(y,x);
				p1.r = color[2];
				p1.g = color[1];
				p1.b = color[0];
			}
		}

// 		cv::Mat_<uint8_t> vis;

// 		cv::convertScaleAbs(weight_upper, vis, 255, 0);
// 		cv::imwrite("/tmp/weight_upper.png", vis);
//
// 		cv::convertScaleAbs(weight_lower, vis, 255, 0);
// 		cv::imwrite("/tmp/weight_lower.png", vis);
//
// 		cv::convertScaleAbs(weight_stereo, vis, 255, 0);
// 		cv::imwrite("/tmp/weight_stereo.png", vis);

		auto end = clock::now();
		NODELET_DEBUG("output voting took %d ms", milliseconds(end - start));
	}

	// Publish the result
	{
		auto start = clock::now();
		m_pub_cloud.publish(output);
		auto end = clock::now();
		NODELET_DEBUG("publishing took %d ms", milliseconds(end - start));
	}

	Cloud::Ptr correctedStereo(new Cloud);
	correctedStereo->header = cloud_upper.header;
	correctedStereo->resize(depth_upper.rows*depth_upper.cols);
	correctedStereo->width = depth_upper.cols;
	correctedStereo->height = depth_upper.rows;

	for(int y = 0; y < depth_stereo.rows; ++y)
	{
		for(int x = 0; x < depth_stereo.cols; ++x)
		{
			float stereo = depth_stereo(y,x);

			Point& p1 = (*correctedStereo)[y * correctedStereo->width + x];

#warning Please check if the upscaling (2*) is correct here
			p1.x = stereo * (SUBSAMPLE*x - m_model_upper.cx()) / m_model_upper.fx();
			p1.y = stereo * (SUBSAMPLE*y - m_model_upper.cy()) / m_model_upper.fy();
			p1.z = stereo;

			auto color = rgb(y,x);
			p1.r = color[2];
			p1.g = color[1];
			p1.b = color[0];
		}
	}

	m_pub_correctedStereo.publish(correctedStereo);

	NODELET_DEBUG("processing finished.");
}

bool DepthFusion::calibrateDepthMaps(const cv::Mat_<float>& source, const cv::Mat_<float>& target, Eigen::Vector2f* result)
{
	assert(source.size() == target.size());
	const int NUM_PIXELS = source.rows * source.cols;

	Eigen::Matrix<float, Eigen::Dynamic, 2> A(NUM_PIXELS, 2);
	Eigen::VectorXf b(NUM_PIXELS);

	int numPoints = 0;

	for(int y = 0; y < source.rows; ++y)
	{
		for(int x = 0; x < source.cols; ++x)
		{
			float valSource = source(y,x);
			float valTarget = target(y,x);

			// Need both values valid
			if(!std::isfinite(valSource) || !std::isfinite(valTarget))
				continue;

			// Discard obviously invalid / uninteresting points
			if(valSource > 1.5 || valTarget > 1.5 || valSource < 0.2 || valTarget < 0.2)
				continue;

			// Discard outliers
			if(std::abs(valSource - valTarget) > 0.03)
				continue;

			// save calibration sample
			A(numPoints, 0) = 1.0f;
			A(numPoints, 1) = valSource;
			b(numPoints) = valTarget;
			numPoints++;
		}
	}

	NODELET_INFO("%d calibration points", numPoints);
	const int MAX_POINTS = 5000;

	if(numPoints < MAX_POINTS)
	{
		NODELET_WARN("not enough points");
		return false;
	}

	if(numPoints > MAX_POINTS)
	{
		NODELET_INFO("resampling to %d points", MAX_POINTS);
		int incr = numPoints / MAX_POINTS;

		int sourceIdx = 0;
		for(int i = 0; i < MAX_POINTS; ++i)
		{
			A.row(i) = A.row(sourceIdx);
			b.row(i) = b.row(sourceIdx);
			sourceIdx += incr;
		}

		numPoints = MAX_POINTS;
	}

	auto svd = A.topRows(numPoints).jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	*result = svd.solve(b.topRows(numPoints));

	NODELET_INFO_STREAM("Least-squares result: " << result->transpose());

	return true;
}

}

PLUGINLIB_EXPORT_CLASS(nimbro_realsense::DepthFusion, nodelet::Nodelet)
