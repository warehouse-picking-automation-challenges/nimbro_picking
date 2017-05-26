// Grasp planning part
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_perception/perception_nodelet.h>

#include <apc_capture/BoxCoordinates.h>

#include <apc_shelf_model/dimensions.h>

#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>

#include <pcl_ros/point_cloud.h>

#include <pcl/features/integral_image_normal.h>

#include <visualization_msgs/Marker.h>

#include <pcl/common/pca.h>

#include "contrib/connectedcomponents.h"

static const float tote_X = 0.30f, tote_Y = 0.18f, tote_Z = 0.2f;

namespace apc_perception
{

std::vector<geometry_msgs::PoseStamped>  PerceptionNodelet::findGraspPose(
	const PointCloudXYZRGB::ConstPtr& cloud,
	const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
	const Eigen::Affine3f& camPoints2Shelf,
	cv::Rect boundingBox, const cv::Mat_<uint8_t>& objectMask,
	const apc_objects::APCObject& info,
	float floorZ, bool* objectStanding)
{
	// Check if the bounding box lies inside the image dimension.
	// If not, modify the bounding box to fit the image dimension.
	capBoundingBox(boundingBox, cloud->width, cloud->height);
	Eigen::Vector3f objectPrincipalAxis;

	// Calculate "robust" height measure
	// copy the points that belongs to the object and project them to ground plane
	float objectHeight;
	{
		std::vector<float> height;

		for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
		{
			for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
			{
				if(!objectMask(y,x))
					continue;


				auto &p = (*m_cloudInContainer)(x, y);

				if (!pcl::isFinite(p))
					continue;

				height.push_back(p.z - floorZ);
			}
		}

		std::sort(height.begin(), height.end());

		if(height.empty())
		{
			NODELET_WARN("object height could not be calculated");
			objectHeight = NAN;
		}
		else
			objectHeight = height[0.9*height.size()];

		NODELET_INFO("object height: %f", objectHeight);
	}

	Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();

	// do PCA on the object cloud to determine a good grasp roll rotation
	{
		// compute the principle axis on the point clound projected on the tote ground plane
		PointCloudXYZRGB::Ptr objectCloud (new PointCloudXYZRGB());
		objectCloud->reserve(m_cloudInContainer->size());

		for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
		{
			for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
			{
				if(!objectMask(y,x))
					continue;

				auto &p = (*m_cloudInContainer)(x, y);

				if (!pcl::isFinite(p))
					continue;

				// get a copy of the point
				auto point = (*m_cloudInContainer)(x, y);

				// project to the ground plane & add it to the list of the object point
				point.z = 0;
				objectCloud->push_back(point);
			}
		}

		if(objectCloud->size() > 3)
		{
			typedef pcl::PointXYZRGB Point;
			Eigen::Matrix3f objectPCA;
			pcl::PCA<Point> pca (true);
			pca.setInputCloud(objectCloud);

			// get the pricipal axis of the object segment
			objectPCA = pca.getEigenVectors();
			objectPrincipalAxis = objectPCA.col(0);

			NODELET_INFO_STREAM("PCA result:\n" << objectPCA);
		}
		else
		{
			NODELET_WARN("Less than three object points");
			objectPrincipalAxis = Eigen::Vector3f::UnitY();
		}

		// transform result into cloud frame
		objectPrincipalAxis = camPoints2Shelf.rotation().transpose() * objectPrincipalAxis;
	}

	std::vector<apc_objects::APCObject::HeuristicGrasp> grasps;

	if(info.heuristicGrasps.empty())
	{
		if(m_mode == MODE_SHELF)
		{
			// Large objects should be grasped at the center
			if(objectHeight < 0.08)
			{
				grasps.push_back(apc_objects::APCObject::HeuristicGrasp::Top);
				grasps.push_back(apc_objects::APCObject::HeuristicGrasp::Center);
			}
			else
			{
				grasps.push_back(apc_objects::APCObject::HeuristicGrasp::Center);
				grasps.push_back(apc_objects::APCObject::HeuristicGrasp::Top);
			}
		}
		else
		{
			// In the tote, prefer center grasps
			grasps.push_back(apc_objects::APCObject::HeuristicGrasp::Center);
			grasps.push_back(apc_objects::APCObject::HeuristicGrasp::Top);
		}
	}
	else
		grasps = info.heuristicGrasps;

	NODELET_INFO("Considered grasp types:");
	for(auto& grasp : grasps)
	{
		switch(grasp)
		{
			case apc_objects::APCObject::HeuristicGrasp::Center: NODELET_INFO(" - center"); break;
			case apc_objects::APCObject::HeuristicGrasp::Top:    NODELET_INFO(" - top");    break;
		}
	}

	// FIXME: Consider second grasp type as well
	if(info.name == "rolodex_jumbo_pencil_cup")
	{
		if(objectHeight < 0.6 * 0.01 * info.standingHeight)
		{
			NODELET_INFO("Cup is not standing, providing side pose");

			float maxDist = -std::numeric_limits<float>::infinity();
			Eigen::Vector3f maxDistPoint;

			float minDist = std::numeric_limits<float>::infinity();
			Eigen::Vector3f minDistPoint;

			// the axis should point towards us
			Eigen::Vector3f axis;
			if(m_mode == MODE_SHELF)
				axis = camPoints2Shelf.rotation().transpose() * (-Eigen::Vector3f::UnitX());
			else
				axis = objectPrincipalAxis;

			for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
			{
				for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
				{
					if(!objectMask(y,x))
						continue;
					auto &p = (*cloud)(x, y);

					if (!pcl::isFinite(p))
						continue;

					float dist = axis.dot(p.getVector3fMap());
					if(dist > maxDist)
					{
						maxDist = dist;
						maxDistPoint = p.getVector3fMap();
					}
					if(dist < minDist)
					{
						minDist = dist;
						minDistPoint = p.getVector3fMap();
					}
				}
			}

			Eigen::Vector3f minDistPointInContainer = camPoints2Shelf * minDistPoint;
			Eigen::Vector3f maxDistPointInContainer = camPoints2Shelf * maxDistPoint;

			minDistPointInContainer.z() = maxDistPointInContainer.z() = std::max(0.05f, objectHeight / 2.0f) + floorZ;

			// Which of the two points is better? Tote: closer to center. Shelf: closer to camera.
			if(
				((m_mode == MODE_TOTE) && minDistPointInContainer.squaredNorm() < maxDistPointInContainer.squaredNorm())
				|| ((m_mode == MODE_SHELF) && minDistPoint.squaredNorm() < maxDistPoint.squaredNorm())
			)
			{
				position = camPoints2Shelf.inverse() * minDistPointInContainer;
				normal = -axis;
			}
			else
			{
				position = camPoints2Shelf.inverse() * maxDistPointInContainer;
				normal = axis;
			}

			NODELET_INFO_STREAM("side pose: " << position.transpose() << ", normal " << normal.transpose());

			if(objectStanding)
				*objectStanding = false;
		}
		else
		{
			NODELET_INFO("Cup is standing, providing top pose");

			Eigen::Array2f minXY(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
			Eigen::Array2f maxXY(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
			std::vector<float> xValues;

			for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
			{
				for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
				{
					if(!objectMask(y,x))
							continue;
					auto &p = (*m_cloudInContainer)(x, y);

					if (!pcl::isFinite(p))
							continue;

					Eigen::Array2f xy = p.getVector3fMap().head<2>();
					minXY = minXY.min(xy);
					maxXY = maxXY.max(xy);

					xValues.push_back(xy.x());
				}
			}

			if(!minXY.allFinite())
			{
				NODELET_WARN("No grasp points after top grasp filtering");
				return std::vector<geometry_msgs::PoseStamped>();
			}

			Eigen::Vector2f graspPosXY = (minXY + maxXY)/2;

			// In the shelf, we mostly see the front surface
			if(m_mode == MODE_SHELF)
			{
				std::sort(xValues.begin(), xValues.end());
				graspPosXY.x() = xValues[0.3 * xValues.size()] + 0.04;
			}

			// Determine robust grasp height (kidding)
			float height = floorZ;

			NODELET_INFO("Determined grasp height: %f (at pos %f, %f)", height, graspPosXY.x(), graspPosXY.y());

			Eigen::Vector3f pos;
			pos << graspPosXY, height;
			position = camPoints2Shelf.inverse() * pos;

			// Do not use the ICP result for the orientation. We always want
			// to suck from above in the *world* frame.
			normal = m_kinematicCamera2World.rotation().transpose() * Eigen::Vector3f::UnitZ();

			if(objectStanding)
				*objectStanding = true;
		}
	}
	else if((info.name == "dasani_water_bottle" || info.name == "elmers_washable_no_run_school_glue")
		&& (objectHeight > 0.8 * 0.01 * info.standingHeight))
	{
		//Measured height is approximatly 90% of the info.standingHeight, plus some relaxation of the threshold
		NODELET_INFO("Water bottles is standing, providing side pose as mean of XY and normal pointing towards camera");
		double meanX = 0;
		double meanY = 0;
		double validPoints = 0;

		for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y){
			for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x){
				if(!objectMask(y,x))
						continue;
				auto &p = (*m_cloudInContainer)(x, y);
				if (!pcl::isFinite(p))
						continue;
				Eigen::Array2f xy = p.getVector3fMap().head<2>();
				meanX += xy[0];
				meanY += xy[1];
				validPoints++;
			}
		}

		if(validPoints == 0)
		{
			NODELET_WARN("No grasp points after top grasp filtering");
			return std::vector<geometry_msgs::PoseStamped>();
		}

		meanX/=validPoints;
		meanY/=validPoints;

		//radius measured from the real item 3cm * 0.01 = 0.03m
		double radius = 0.03;
		normal = Eigen::Vector3f(-meanX, -meanY,0);
		normal.normalize();

		float z = 0.45 * (0.01 * info.standingHeight);

		Eigen::Vector3f pos(meanX , meanY, z); // 5cm from floor
		NODELET_INFO("standing height: %f => height %f", info.standingHeight, pos.z());
		pos = pos + normal * radius;

		normal.z() += std::sin(30.0f * M_PI / 180.0f);
		normal.normalize();

		// transform the point into the camera frame
		position = camPoints2Shelf.inverse() * pos;

		//Just pointing to the middle of the box
		normal = camPoints2Shelf.rotation().transpose() * normal;

		if(objectStanding)
			*objectStanding = true;
	}
	else if(grasps.front() == apc_objects::APCObject::HeuristicGrasp::Top)
	{
		// Top grasp!
		Eigen::Vector2f graspPosXY = Eigen::Vector2f::Zero();

		if(info.topGraspHigh)
		{
			float weightSum = 0.0f;

			for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
			{
				for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
				{
					if(!objectMask(y,x))
						continue;
					auto &p = (*m_cloudInContainer)(x, y);

					if (!pcl::isFinite(p))
						continue;

					auto inShelf = p.getVector3fMap();
					float h = inShelf.z() - floorZ;
					if (h >= 0.7 * objectHeight)
					{
						auto coord = Eigen::Vector2f(inShelf.x(), inShelf.y());
						graspPosXY += h * coord;
						weightSum += h;
					}
				}
			}

			graspPosXY /= weightSum;
		}
		else
		{
			Eigen::Array2f minXY(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
			Eigen::Array2f maxXY(-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());

			for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
			{
				for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
				{
					if(!objectMask(y,x))
						continue;
					auto &p = (*m_cloudInContainer)(x, y);

					if (!pcl::isFinite(p))
						continue;

					Eigen::Array2f xy = p.getVector3fMap().head<2>();
					minXY = minXY.min(xy);
					maxXY = maxXY.max(xy);
				}
			}

			if(!minXY.allFinite())
			{
				NODELET_WARN("No grasp points after top grasp filtering");
				return std::vector<geometry_msgs::PoseStamped>();
			}

			graspPosXY = (minXY + maxXY)/2;

			NODELET_INFO_STREAM("initial cloud center: " << graspPosXY.transpose());

			float minDist2 = std::numeric_limits<float>::infinity();
			Eigen::Vector2f closest = graspPosXY;

			// Find the closest point in the cloud to this point
			for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
			{
				for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
				{
					if(!objectMask(y,x))
						continue;

					auto p = (*m_cloudInContainer)(x, y).getVector3fMap();

					float dist = (p.head<2>() - graspPosXY).squaredNorm();
					if(dist < minDist2)
					{
						minDist2 = dist;
						closest = p.head<2>();
					}
				}
			}

			graspPosXY = closest;
			NODELET_INFO_STREAM("initial closest point: " << graspPosXY.transpose());
		}

		// Determine robust grasp height
		constexpr float GRASP_RADIUS = 0.03;
		std::vector<float> graspHeights;

		for(int y =  boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
		{
			for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
			{
				if(!objectMask(y,x))
					continue;

				auto &p = (*m_cloudInContainer)(x, y);

				if (!pcl::isFinite(p))
					continue;

				auto dist = p.getVector3fMap().head<2>() - graspPosXY;
				if (dist.squaredNorm() < GRASP_RADIUS * GRASP_RADIUS)
				{
					graspHeights.push_back(p.z);
				}
			}
		}

		float height = 0.0;
		if(!graspHeights.empty())
			height = graspHeights[0.9*graspHeights.size()];

		NODELET_INFO("Determined grasp height: %f (at pos %f, %f)", height, graspPosXY.x(), graspPosXY.y());

		// Re-determine grasp position
		Eigen::Vector2f refinedGraspXY = Eigen::Vector2f::Zero();
		unsigned int graspPoints = 0;

		for(int y =  boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
		{
			for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
			{
				if(!objectMask(y,x))
					continue;

				auto &p = (*m_cloudInContainer)(x, y);

				if (!pcl::isFinite(p))
					continue;

				auto dist = p.getVector3fMap().head<2>() - graspPosXY;
				if (dist.squaredNorm() < GRASP_RADIUS * GRASP_RADIUS && std::abs(p.z - height) < 0.002)
				{
					refinedGraspXY += p.getVector3fMap().head<2>();
					graspPoints++;
				}
			}
		}

		if(graspPoints != 0)
		{
			refinedGraspXY /= graspPoints;

			Eigen::Vector3f pos;
			pos << refinedGraspXY, height;
			position = camPoints2Shelf.inverse() * pos;

			// Do not use the ICP result for the orientation. We always want
			// to suck from above in the *world* frame.
			normal = m_kinematicCamera2World.rotation().transpose() * Eigen::Vector3f::UnitZ();
		}
		else
			position << NAN, NAN, NAN;
	}
	else
	{
		// center grasp!
		int SUPPORT_RADIUS = 5;
		int center_y = boundingBox.y + boundingBox.height/2;
		int center_x = boundingBox.x + boundingBox.width/2;
		int end_y = center_y + SUPPORT_RADIUS;
		int end_x = center_x + SUPPORT_RADIUS;

		int numValid = 0;

		for(int y = std::max<int>(center_y - SUPPORT_RADIUS, 0); y < end_y; ++y)
		{
			for(int x = std::max<int>(center_x - SUPPORT_RADIUS, 0); x < end_x; ++x)
			{
				if(!objectMask(y,x))
					continue;

				auto& p = (*cloud)(x,y);
				auto& n = (*normals)(x,y);
				if(!pcl::isFinite(p) || !pcl::isFinite(n))
					continue;

// 				NODELET_INFO_STREAM("adding normal: " << n.getNormalVector3fMap().transpose());

				position += p.getVector3fMap();
				normal += n.getNormalVector3fMap();
				numValid++;
			}
		}

		position /= numValid;
		normal /= numValid;
		normal.normalize();

		NODELET_INFO_STREAM("smoothed normal: " << normal.transpose());

		// Convert position, normal into shelf frame for some checks
		if(m_mode == MODE_SHELF)
		{
			position = camPoints2Shelf * position;
			normal = camPoints2Shelf.rotation() * normal;

			// Prevent normal from pointing downwards (no application for that)
			if(normal.z() < 0.0f)
				normal.z() = 0.0f;
			normal.normalize();

			// Don't suck too low.
			constexpr float MIN_HEIGHT_ABOVE_PLANE = 0.07f;
			const float lowerLimit = floorZ + MIN_HEIGHT_ABOVE_PLANE;
			if(position.z() < lowerLimit)
			{
				// Is the object high enough to suck higher?
				if(objectHeight >= MIN_HEIGHT_ABOVE_PLANE)
					position.z() = lowerLimit;
				else
				{
					// Suck from above
					normal = -(Eigen::AngleAxisf(M_PI/8.0, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitX());
					position.z() = std::max(position.z(), floorZ + 0.02f);
				}
			}

			// If the normal is almost straight in the XY plane, then force it
			// to be straight.
			if(std::abs(std::atan2(-normal.y(), -normal.x())) < 15.0 * M_PI / 180.0)
			{
				// If the pitch angle is also close to zero, force it as well
				if(std::abs(std::atan2(-normal.z(), -normal.x())) < 15.0 * M_PI / 180.0)
				{
					// just a slight pitch downward to keep the IK happy
					normal = -(Eigen::AngleAxisf(M_PI/32.0, Eigen::Vector3f::UnitY()) * Eigen::Vector3f::UnitX());
				}
				else
				{
					// use original Z
					normal.x() = -1.0;
					normal.y() = 0.0;
					normal.normalize();
				}
			}

			position = camPoints2Shelf.inverse() * position;
			normal = camPoints2Shelf.rotation().transpose() * normal;

			// Limit again in world frame for safety reasons (in case ICP is bad)
			if(m_mode == MODE_SHELF)
			{
				position = m_kinematicCamera2World * position;
				normal = m_kinematicCamera2World.rotation() * normal;

				float minZ = apc_shelf_model::getBoxMinZ(m_act_goal->box_row);
				NODELET_INFO("Grasp Z in world frame: %f, lower front box edge is at %f", position.z(), minZ);

				if(position.z() < minZ + 0.03)
				{
					NODELET_WARN("ICP seems to be bad, re-limiting lower grasp position");
					position.z() = minZ + 0.03;
				}

				position = m_kinematicCamera2World.inverse() * position;
				normal = m_kinematicCamera2World.rotation().transpose() * normal;
			}
		}
		else
		{
			position = camPoints2Shelf * position;
			normal = camPoints2Shelf.rotation() * normal;

			Eigen::Vector3f graspPose = position + normal * 0.06;
			if(std::abs(graspPose.x()) >= tote_X || std::abs(graspPose.y()) >= tote_Y)
			{
				NODELET_ERROR("Grasp position is outside of tote");
				position << NAN, NAN, NAN;
			}

			// If the normal is close to +Z, snap to +Z
			if(normal.z() > cos(30.0 * M_PI / 180.0))
			{
				NODELET_INFO("normal is close to +Z, snapping to +Z (angle: %f deg)",
					acos(normal.z()) * 180.0 / M_PI
				);
				normal = Eigen::Vector3f::UnitZ();
			}

			position = camPoints2Shelf.inverse() * position;
			normal = camPoints2Shelf.rotation().transpose() * normal;
		}
	}

	NODELET_INFO_STREAM("position: " << position.transpose() << ", normal: " << normal.transpose());

	if(!position.allFinite())
	{
		NODELET_WARN("Grasp computation failed.");
		return std::vector<geometry_msgs::PoseStamped>();
	}

	Eigen::Affine3f graspPose;
	{
		// use the computed pricipal axis of the object segment to get the grasp pose

		// inverse of the normal should be aligned with the x axis
		Eigen::Matrix3f rot;
		rot.col(0) = -normal; // X points towards the object

		// check if the principal axis is closer to the normal
		// if the angle b/w normal and principal axis is less that 10 degress, then go with the old code ( align X axis with normal and choose Y and Z accordingly)
		// else align X with normal and Y with principal axis
		if (std::abs(normal.dot(objectPrincipalAxis)) > std::cos(10 * M_PI / 180))
		{
			// make sure that the y axis is orthogonal to x axis
			if(fabs(rot.col(0).z()) > 0.5)
				rot.col(1) = Eigen::Vector3f::UnitY().cross(-normal).normalized();
			else
				rot.col(1) = Eigen::Vector3f::UnitZ().cross(-normal).normalized();

			// Choose Z axis
			rot.col(2) = rot.col(0).cross(rot.col(1));
		}
		else
		{
			// we need Z axis of the object to be aligned with principal axis
			// thus we need a rotation around X such that X is the inverse of the normal
			// and Z is closer to the principal axis of the object segment

			NODELET_INFO_STREAM("normal: " << normal.transpose());
			NODELET_INFO_STREAM("objectPrincipalAxis: " << objectPrincipalAxis.transpose());

			Eigen::Vector3f tempPerpendicularAxis = normal.cross(objectPrincipalAxis).normalized();

			NODELET_INFO_STREAM("tempPerpendicularAxis: " << tempPerpendicularAxis.transpose());

			rot.col(2) = tempPerpendicularAxis.cross(normal).normalized();

			// If we are grasping "backwards", rotate the grasp by 180°.
			if(rot.col(2).x() < 0)
				rot.col(2) = -rot.col(2);

			NODELET_INFO_STREAM("result: " << rot.col(1).transpose());

			// Choose Y axis
			rot.col(1) = rot.col(2).cross(rot.col(0));
		}

		graspPose = Eigen::Translation3f(position) * rot;
	}

	// Marker visualization
	{
		visualization_msgs::Marker marker;
		marker.action = marker.ADD;
		marker.header = pcl_conversions::fromPCL(cloud->header);


		marker.type = marker.ARROW;

		Eigen::Vector3f startPoint = graspPose * Eigen::Vector3f(-0.1, 0, 0);
		Eigen::Vector3f endPoint = graspPose * Eigen::Vector3f(0, 0, 0);

		marker.points.resize(2);
		tf::pointEigenToMsg(startPoint.cast<double>(), marker.points[0]);
		tf::pointEigenToMsg(endPoint.cast<double>(), marker.points[1]);

		marker.scale.x = 0.01;
		marker.scale.y = 0.02;
		marker.scale.z = 0.0;

		marker.color.a = 1.0;
		marker.color.r = 1.0;

		m_pub_marker.publish(marker);

		geometry_msgs::PoseStamped pose;
		pose.header = pcl_conversions::fromPCL(cloud->header);

		tf::poseEigenToMsg(graspPose.cast<double>(), pose.pose);
		m_pub_pose.publish(pose);
	}

	std::vector<geometry_msgs::PoseStamped> result(1);
	//apc_perception::ApcPerceptionResult result;
	//result.grasp_poses.resize(1);

	//result.grasp_poses[0].header = pcl_conversions::fromPCL(cloud->header);
	//tf::poseEigenToMsg(graspPose.cast<double>(), result.grasp_poses[0].pose);

	tf::poseEigenToMsg(graspPose.cast<double>(), result[0].pose);
	result[0].header = pcl_conversions::fromPCL(cloud->header);
	// Transform pose into world link
	if(m_tf->waitForTransform("world", cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), ros::Duration(5.0)))
	{
		tf::StampedTransform transform;
		m_tf->lookupTransform("world", cloud->header.frame_id, pcl_conversions::fromPCL(cloud->header.stamp), transform);

		Eigen::Affine3d eigenTransform;
		tf::transformTFToEigen(transform, eigenTransform);

		Eigen::Affine3d graspInWorld = eigenTransform * graspPose.cast<double>();

		geometry_msgs::Pose msgPose;
		tf::poseEigenToMsg(graspInWorld, msgPose);

		NODELET_INFO("graspInWorld: [%f %f %f], rot [%f %f %f %f]\n",
			msgPose.position.x, msgPose.position.y, msgPose.position.z,
			msgPose.orientation.x, msgPose.orientation.y, msgPose.orientation.z, msgPose.orientation.w
		);
	}

	//result.box_coordinates = m_shelfFeatures.getBoxCoordinates();
	//result.grasp_object = objectName;

	//m_act_perceptionServer->setSucceeded(result);
	//m_act_goal.reset();

	//m_sub_pointCloud.shutdown();
	return result;
}
std::vector<geometry_msgs::PoseStamped> PerceptionNodelet::findShelfGraspPose(
	const PointCloudXYZRGB::ConstPtr& cloud,
	const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
	const Eigen::Affine3f& camPoints2Shelf,
	cv::Rect boundingBox, const cv::Mat_<uint8_t>& objectMask,
	const apc_objects::APCObject& info, bool* objectStanding)
{
	cv::Mat_<uint8_t> prunedObjectMask;
	objectMask.copyTo(prunedObjectMask);

	auto coord = m_shelfFeatures.getBoxCoordinates();

	for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
	{
		for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
		{
			if(!objectMask(y,x))
				continue;

			auto &p = (*m_cloudInContainer)(x, y);

			if (!pcl::isFinite(p))
			{
				prunedObjectMask(y,x) = 0;
				continue;
			}

			auto inShelf = p.getVector3fMap();
			float dist2Walls = std::min({
				coord.boxCoordinates[1] - inShelf.x(),

				inShelf.y() - coord.boxCoordinates[2],
				coord.boxCoordinates[3] - inShelf.y()
			});

			// Exclude points near the walls
			if(dist2Walls < 0.02)
			{
				prunedObjectMask(y,x) = 0;
				continue;
			}
		}
	}

	float floorZ = coord.boxCoordinates[4];
	return findGraspPose(
		cloud, normals, camPoints2Shelf,
		boundingBox, prunedObjectMask, info,
		floorZ, objectStanding
	);
}


bool PerceptionNodelet::refineMaskforSqueaky(cv::Mat_<uint8_t>& objectMask)
{
	// For sqeaking eggs look for distinctive color regions
	// logic : prefer blue region more and in case of blue region occulusion, look for orange and yellow is least prefered
	// classify the pixels into the color classes in YUV space
	// get the prefered segmentation and do connected components
	// use the bigger connected components

	enum COLOR
	{
		BLUE,
		RED,
		YELLOW,
	};
	std::map<COLOR, cv::Mat_<u_int8_t> > pixelClassMask;

	uint8_t pixelIndices[3];
	pixelIndices[BLUE]     = m_lut.codeForName("blue");
	pixelIndices[RED]      = m_lut.codeForName("orange");
	pixelIndices[YELLOW]   = m_lut.codeForName("yellow");

	pixelClassMask[BLUE]   = cv::Mat_<u_int8_t>::zeros(objectMask.rows,objectMask.cols);
	pixelClassMask[RED]    = cv::Mat_<u_int8_t>::zeros(objectMask.rows,objectMask.cols);
	pixelClassMask[YELLOW] = cv::Mat_<u_int8_t>::zeros(objectMask.rows,objectMask.cols);

	cv::Mat_<u_int8_t>pixelClasses (objectMask.rows,objectMask.cols,255); // 255 is just a value that won't be used by LUT for color id

	for (int y = 0; y < objectMask.rows; ++y)
	{
		for (int x =0; x < objectMask.cols; ++x)
		{
			if (!objectMask(y,x))
				continue;

			// get the YUV based classification result
			auto rgb=this->m_featureImages["rgb"].at<cv::Vec3b>(y,x);
			uint8_t Y = ( (  66 * rgb[2] + 129 * rgb[1] +  25 * rgb[0] + 128) >> 8) +  16;
			uint8_t U = ( ( -38 * rgb[2] -  74 * rgb[1] + 112 * rgb[0] + 128) >> 8) + 128;
			uint8_t V = ( ( 112 * rgb[2] -  94 * rgb[1] -  18 * rgb[0] + 128) >> 8) + 128;
			pixelClasses(y,x) = m_lut.classify(Y,U,V);
		}
	}

	pixelClassMask[BLUE].setTo(1, pixelClasses == pixelIndices[BLUE] );
	pixelClassMask[RED].setTo(1, pixelClasses == pixelIndices[RED] );
	pixelClassMask[YELLOW].setTo(1, pixelClasses == pixelIndices[YELLOW] );

	// count the pixels for each class

	double noOfBlues = cv::sum(pixelClassMask[BLUE])[0];
	double noOfReds = cv::sum(pixelClassMask[RED])[0];
	double noOfYellows = cv::sum(pixelClassMask[YELLOW])[0];

	cv::Mat_<u_int8_t> refinedObjectMask= cv::Mat_<u_int8_t>::zeros(objectMask.rows,objectMask.cols);

	if (noOfBlues > 0.25 * (noOfReds +noOfYellows))
		refinedObjectMask = pixelClassMask[BLUE];
	else
	{
		if (noOfReds > 0.5 * noOfYellows )
			refinedObjectMask = pixelClassMask[RED];
		else
			refinedObjectMask = pixelClassMask[YELLOW];
	}


	// do connected components

	NODELET_INFO("Connected components...");
	cv::Mat_<int32_t> labels;
	cv::Mat_<int32_t> stats;
	cv::Mat_<double> centroids;

	int num_labels = ::connectedComponentsWithStats(refinedObjectMask, labels, stats, centroids, 4, CV_32S);

	if(num_labels <= 1)
	{
		NODELET_INFO("No segments :-(");
		return false;
	}

	// Find biggest component
	int maxArea = 0;
	int maxIdx = -1;

	for(int i = 1; i < num_labels; ++i)
	{
		int area = stats(i, CC_STAT_AREA);
		if(area > maxArea)
		{
			maxArea = area;
			maxIdx = i;
		}
	}
	refinedObjectMask.setTo(1,labels == maxIdx );


	cv::imwrite("/tmp/squeaky_segm.png", pixelClasses);
	cv::imwrite("/tmp/squeaky_refined.png", 255 * refinedObjectMask);
	cv::imwrite("/tmp/squeaky_initial.png", objectMask);

	//dilate and erode by 60 pixels (closing operator to fill the circle)

	const int closing_size = 60;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
												cv::Size(2*closing_size + 1, 2*closing_size+1),
												cv::Point(closing_size, closing_size)
	);
	cv::dilate(refinedObjectMask, refinedObjectMask, element);
	cv::erode(refinedObjectMask, refinedObjectMask, element);
	
	objectMask = refinedObjectMask;
	return true;

}

std::vector<geometry_msgs::PoseStamped> PerceptionNodelet::findToteGraspPose(
	const PointCloudXYZRGB::ConstPtr& cloud,
	const pcl::PointCloud<pcl::Normal>::ConstPtr& normals,
	const Eigen::Affine3f& camPoints2Shelf,
	cv::Rect boundingBox, const cv::Mat_<uint8_t>& objectMask,
	const apc_objects::APCObject& info, bool* objectStanding)
{

	float floorZ = 0.0f;

	if (info.name == "kyjen_squeakin_eggs_plush_puppies")
	{
		cv::Mat_<uint8_t> refineMask = objectMask.clone();
		if(!refineMaskforSqueaky(refineMask))
		{
			std::vector<geometry_msgs::PoseStamped> empty;
			return  empty;
		}

		return findGraspPose(
				cloud, normals, camPoints2Shelf,
				boundingBox, refineMask, info,
				floorZ, objectStanding
		);
	}
	else
		return findGraspPose(
			cloud, normals, camPoints2Shelf,
			boundingBox, objectMask, info,
			floorZ, objectStanding
		);
}

std::vector<geometry_msgs::PoseStamped> PerceptionNodelet::findRegistrationGraspPose(
	const PointCloudXYZRGB::ConstPtr& cloud,
	const cv::Mat_<uint8_t>& objectMask,
	const Eigen::Affine3f& cam2Container,
	const apc_objects::APCObject& info, float floorZ)
{
	NODELET_INFO("Registration...");

	if(info.name == "scotch_duct_tape")
	{
		// Calculate "robust" height measure
		// copy the points that belongs to the object and project them to ground plane
		float objectHeight;
		{
			std::vector<float> height;

			for(size_t y = 0; y < cloud->height; ++y)
			{
				for(size_t x = 0; x < cloud->width; ++x)
				{
					if(!objectMask(y,x))
						continue;

					auto &p = (*m_cloudInContainer)(x, y);

					if (!pcl::isFinite(p))
						continue;

					height.push_back(p.z - floorZ);
				}
			}

			std::sort(height.begin(), height.end());

			if(height.empty())
			{
				NODELET_WARN("object height could not be calculated");
				objectHeight = NAN;
			}
			else
				objectHeight = height[0.9*height.size()];

			NODELET_INFO("object height: %f", objectHeight);
		}

		if(objectHeight > 0.01 * info.standingHeight)
		{
			NODELET_INFO("Duct tape is standing, refusing to do registration");
			return std::vector<geometry_msgs::PoseStamped>();
		}
	}

	std::string singleModelPath = ros::package::getPath("apc_6dpose") + "/models/" + info.name;

	m_6Dpose_recog.setUseNaiveApproach(true);
	m_6Dpose_recog.setUseVisibility(false);
	m_6Dpose_recog.setICPType(1);
	m_6Dpose_recog.setPerformDilate(true);
	m_6Dpose_recog.initialize(singleModelPath, singleModelPath + "/" + info.name + "/grasps.txt");
	auto registrationResult = m_6Dpose_recog.compute(cloud, info.name, Eigen::Vector3f(), objectMask);

	NODELET_INFO("Registration distance: %f\n", registrationResult.distance);
	NODELET_INFO_STREAM("Registration result:\n" << registrationResult.objectPose.matrix());

	visualization_msgs::Marker marker;
	marker.action = marker.ADD;
	marker.type = marker.LINE_LIST;

	marker.header.frame_id = cloud->header.frame_id;
	marker.header.stamp = pcl_conversions::fromPCL(cloud->header.stamp);

	marker.scale.x = 0.03;

	marker.color.a = 1.0;
	marker.color.g = 1.0;

	for(size_t i = 0; i < registrationResult.grasping_points.size(); ++i)
	{
		Eigen::Vector3f point = registrationResult.grasping_points[i].getVector3fMap();
		Eigen::Vector3f normal = registrationResult.grasping_directions[i].getVector3fMap();

		Eigen::Vector3f tail = point + 0.1 * normal;

		geometry_msgs::Point start;
		start.x = point.x(); start.y = point.y(); start.z = point.z();
		marker.points.push_back(start);

		geometry_msgs::Point end;
		end.x = tail.x(); end.y = tail.y(); end.z = tail.z();
		marker.points.push_back(end);
	}

	m_pub_registration_grasps.publish(marker);

	m_pub_registration_aligned.publish(registrationResult.aligned_model);

	std::vector<geometry_msgs::PoseStamped> poses;

	for(size_t i = 0; i < registrationResult.grasping_points.size(); ++i)
	{
		Eigen::Vector3f normal = registrationResult.grasping_directions[i].getVector3fMap();

		Eigen::Vector3f normalInContainer = cam2Container.rotation() * normal;
		if(normalInContainer.z() < 0)
			continue;

		Eigen::Affine3f graspPose;
		{
			Eigen::Matrix3f rot;
			rot.col(0) = -normal; // X points towards the object

			if(fabs(rot.col(0).z()) > 0.5)
				rot.col(1) = Eigen::Vector3f::UnitY().cross(-normal).normalized();
			else
				rot.col(1) = Eigen::Vector3f::UnitZ().cross(-normal).normalized();

			rot.col(2) = rot.col(0).cross(rot.col(1));

			graspPose = Eigen::Translation3f(registrationResult.grasping_points[i].getVector3fMap()) * rot;
		}

		Eigen::Affine3f graspInContainer = cam2Container * graspPose;

		Eigen::Vector3f pregraspPoint(-0.1, 0.0, 0.0);
		pregraspPoint = graspInContainer * pregraspPoint;

		if(std::abs(pregraspPoint.x()) >= tote_X || std::abs(pregraspPoint.y()) >= tote_Y || pregraspPoint.z() < 0)
			continue;

		geometry_msgs::PoseStamped pose;
		tf::poseEigenToMsg(graspPose.cast<double>(), pose.pose);

		pose.header = pcl_conversions::fromPCL(cloud->header);

		poses.push_back(pose);
	}

	// Sort by distance to camera
	std::sort(poses.begin(), poses.end(), [](const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b) {
		Eigen::Vector3d va; tf::pointMsgToEigen(a.pose.position, va);
		Eigen::Vector3d vb; tf::pointMsgToEigen(b.pose.position, vb);

		return va.squaredNorm() < vb.squaredNorm();
	});

	if(!poses.empty())
	{
		Eigen::Affine3d graspPoseD;
		tf::poseMsgToEigen(poses[0].pose, graspPoseD);
		Eigen::Affine3f graspPose = graspPoseD.cast<float>();

		visualization_msgs::Marker marker;
		marker.action = marker.ADD;
		marker.header = pcl_conversions::fromPCL(cloud->header);


		marker.type = marker.ARROW;

		Eigen::Vector3f startPoint = graspPose * Eigen::Vector3f(-0.1, 0, 0);
		Eigen::Vector3f endPoint = graspPose * Eigen::Vector3f(0, 0, 0);

		marker.points.resize(2);
		tf::pointEigenToMsg(startPoint.cast<double>(), marker.points[0]);
		tf::pointEigenToMsg(endPoint.cast<double>(), marker.points[1]);

		marker.scale.x = 0.01;
		marker.scale.y = 0.02;
		marker.scale.z = 0.0;

		marker.color.a = 1.0;
		marker.color.r = 1.0;

		m_pub_marker.publish(marker);
	}

	return poses;
}

std::vector<geometry_msgs::PoseStamped> PerceptionNodelet::findShelfRegistrationGraspPose(
	const PointCloudXYZRGB::ConstPtr& cloud,
	cv::Rect boundingBox,
	const cv::Mat_<uint8_t>& objectMask,
	const Eigen::Affine3f& cam2Container,
	const apc_objects::APCObject& info)
{
	cv::Mat_<uint8_t> prunedObjectMask;
	objectMask.copyTo(prunedObjectMask);

	auto coord = m_shelfFeatures.getBoxCoordinates();

	for(int y = boundingBox.y; y < boundingBox.y + boundingBox.height; ++y)
	{
		for(int x = boundingBox.x; x < boundingBox.x + boundingBox.width; ++x)
		{
			if(!objectMask(y,x))
				continue;

			auto &p = (*m_cloudInContainer)(x, y);

			if (!pcl::isFinite(p))
			{
				prunedObjectMask(y,x) = 0;
				continue;
			}

			auto inShelf = p.getVector3fMap();
			float dist2Walls = std::min({
				coord.boxCoordinates[1] - inShelf.x(),

				inShelf.y() - coord.boxCoordinates[2],
				coord.boxCoordinates[3] - inShelf.y()
			});

			// Exclude points near the walls
			if(dist2Walls < 0.02)
			{
				prunedObjectMask(y,x) = 0;
				continue;
			}
		}
	}

	float floorZ = coord.boxCoordinates[4];
	return findRegistrationGraspPose(
		cloud, objectMask, cam2Container, info, floorZ
	);
}

}
