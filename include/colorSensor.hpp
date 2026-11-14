#ifndef COLOR_SENSOR_HPP
#define COLOR_SENSOR_HPP

enum class DetectedColor {
	None,
	Red,
	Orange,
	Green,
	Indigo
};

struct ColorReading {
	int red;
	int green;
	int blue;
	int clear;
};

class ColorSensor {
  public:
	bool begin();
	bool read(ColorReading& reading);
	DetectedColor classify(const ColorReading& reading) const;
};

#endif
