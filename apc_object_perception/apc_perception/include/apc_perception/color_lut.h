// YUV color LUT
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef COLOR_LUT_H
#define COLOR_LUT_H

#include <array>
#include <vector>
#include <cstdint>

class ColorLUT
{
public:
	struct ColorSpec
	{
		uint8_t id;
		std::string name;
		uint8_t minY;
		uint8_t maxY;
	};

	bool loadFromFile(const std::string& filename);

	uint8_t codeForName(const std::string& colorName) const;
	std::string nameForCode(uint8_t code) const;
	uint8_t classify(uint8_t y, uint8_t u, uint8_t v) const;

	const std::vector<ColorSpec>& colors() const
	{ return m_colors; }
private:
	std::array<uint8_t, 256*256> m_lut;
	std::vector<ColorSpec> m_colors;
};

#endif
