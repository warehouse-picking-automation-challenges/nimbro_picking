// YUV color LUT
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <apc_perception/color_lut.h>

#include <fstream>

#include <yaml-cpp/yaml.h>

#include <ros/console.h>

bool ColorLUT::loadFromFile(const std::string& filename)
{
	try
	{
		ROS_INFO("Loading '%s'", filename.c_str());
		YAML::Node doc = YAML::LoadFile(filename);
		if(!doc)
			return false;

		YAML::Node colorSpecs = doc["colors"];
		for(const YAML::Node& color : colorSpecs)
		{
			ColorSpec spec;
			spec.name = color["name"].as<std::string>();
			spec.minY = color["minY"].as<unsigned int>();
			spec.maxY = color["maxY"].as<unsigned int>();
			spec.id = color["id"].as<unsigned int>();

			if(m_colors.size() <= spec.id)
				m_colors.resize(spec.id+1);

			m_colors[spec.id] = spec;
		}

		std::string encodedLUT = doc["classes"].as<std::string>();

		if(encodedLUT.size() != m_lut.size())
		{
			ROS_ERROR("Invalid LUT size: %lu, expected %lu", encodedLUT.length(), m_lut.size());
			return false;
		}

		for(unsigned int i = 0; i < m_lut.size(); ++i)
		{
			uint8_t c = encodedLUT[i];
			if(c == '-')
				m_lut[i] = 0xFF;
			else
				m_lut[i] = c - 0x30;
		}
	}
	catch(YAML::Exception& e)
	{
		ROS_ERROR("Could not load LUT: %s", e.what());
	}

	return true;
}

uint8_t ColorLUT::codeForName(const std::string& colorName) const
{
	for(unsigned int i = 0; i < m_colors.size(); ++i)
	{
		if(m_colors[i].name == colorName)
			return i;
	}

	ROS_ERROR("Unknown color name '%s'", colorName.c_str());
	throw std::runtime_error("Unknown color name");
}

std::string ColorLUT::nameForCode(uint8_t code) const
{
	if(code == 0xFF || code >= m_colors.size())
		return "unknown";

	return m_colors[code].name;
}

uint8_t ColorLUT::classify(uint8_t y, uint8_t u, uint8_t v) const
{
	uint8_t lut = m_lut[256*u + v];

	if(lut == 0xFF)
		return 0xFF;

	if(lut >= m_colors.size())
	{
		ROS_WARN("Invalid LUT entry");
		return 0xFF;
	}

	const auto& color = m_colors[lut];

	if(y < color.minY || y > color.maxY)
		return 0xFF;

	return lut;
}
