// Qt Item model for annotation masks
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "annotation_model.h"

#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <QFile>

AnnotationModel::AnnotationModel(QObject* parent)
 : QAbstractListModel(parent)
{
}

AnnotationModel::~AnnotationModel()
{
}

int AnnotationModel::rowCount(const QModelIndex& parent) const
{
	if(parent.isValid())
		return 0;

	return m_items.count();
}

QVariant AnnotationModel::data(const QModelIndex& index, int role) const
{
	if(index.row() < 0 || index.row() >= m_items.count())
		return QVariant();

	const AnnotationItem& item = m_items[index.row()];

	switch(role)
	{
		case Qt::DisplayRole:
			return item.name;
		default:
			return QVariant();
	}
}

void AnnotationModel::addItem(const QString& name, const std::vector<cv::Point>& polygon)
{
	AnnotationItem item;
	item.name = name;
	item.polygon = polygon;

	beginInsertRows(QModelIndex(), m_items.count(), m_items.count());
	m_items.push_back(item);
	endInsertRows();
}

void AnnotationModel::clear()
{
	beginResetModel();
	m_items.clear();
	endResetModel();
}

bool AnnotationModel::loadFrom(const QString& filename)
{
	beginResetModel();
	m_items.clear();

	try
	{
		YAML::Node doc = YAML::LoadFile(filename.toStdString());

		YAML::Node polygons = doc["polygons"];

		for(std::size_t i = 0; i < polygons.size(); ++i)
		{
			AnnotationItem item;
			item.name = QString::fromStdString(polygons[i]["name"].as<std::string>());

			YAML::Node points = polygons[i]["points"];
			for(std::size_t j = 0; j < points.size(); ++j)
			{
				item.polygon.emplace_back(
					points[j][0].as<int>(),
					points[j][1].as<int>()
				);
			}

			m_items.push_back(item);
		}
	}
	catch(YAML::Exception& e)
	{
		std::cerr << "Got YAML exception: " << e.what() << "\n";

		endResetModel();
		return false;
	}

	endResetModel();
	return true;
}

void AnnotationModel::save(const QString& filename)
{
	if(m_items.empty())
	{
		QFile::remove(filename);
		return;
	}

	std::ofstream out(filename.toStdString());
	YAML::Emitter em(out);

	em << YAML::BeginMap << YAML::Key << "polygons" << YAML::Value << YAML::BeginSeq;

	for(const auto& item : m_items)
	{
		em << YAML::BeginMap;

		em << YAML::Key << "name" << YAML::Value << item.name.toStdString();

		em << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;

		for(const auto& point : item.polygon)
		{
			em << YAML::BeginSeq << point.x << point.y << YAML::EndSeq;
		}

		em << YAML::EndSeq;

		em << YAML::EndMap;
	}

	em << YAML::EndSeq << YAML::EndMap;
}

void AnnotationModel::saveMasks(const QString& dir, int width, int height)
{
	cv::Mat_<uint8_t> mask(height, width);
	cv::Mat_<uint8_t> bgMask(height, width, (uint8_t)255);

	std::map<std::string, std::vector<const AnnotationItem*>> map;

	for(const AnnotationItem& item : m_items)
		map[item.name.toStdString()].push_back(&item);

	// If we have a box annotation, mark it as 255 in the background mask
// 	auto it = map.find("box");
// 	if(it != map.end())
// 	{
// 		for(const AnnotationItem* item : it->second)
// 		{
// 			const cv::Point* pts = item->polygon.data();
// 			int npts = item->polygon.size();
//
// 			cv::fillPoly(bgMask, &pts, &npts, 1, cv::Scalar(255));
// 		}
// 	}

	for(auto& pair : map)
	{
		mask = 0;

		for(const AnnotationItem* item : pair.second)
		{
			const cv::Point* pts = item->polygon.data();
			int npts = item->polygon.size();
			cv::fillPoly(mask, &pts, &npts, 1, cv::Scalar(255));

			if(pair.first != "box")
				cv::fillPoly(bgMask, &pts, &npts, 1, cv::Scalar(0));
		}

		cv::imwrite(dir.toStdString() + "/mask_" + pair.first + ".png", mask);
	}

	cv::imwrite(dir.toStdString() + "/mask_container.png", bgMask);
}

void AnnotationModel::deleteItem(int index)
{
	beginRemoveRows(QModelIndex(), index, index);
	m_items.removeAt(index);
	endRemoveRows();
}
