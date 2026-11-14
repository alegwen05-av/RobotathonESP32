#include "../include/colorSensor.hpp"
#include <Arduino_APDS9960.h>
#include <algorithm>

bool ColorSensor::begin() {
	return APDS.begin();
}

bool ColorSensor::read(ColorReading& reading) {
	if (!APDS.colorAvailable()) {
		return false;
	}

	return APDS.readColor(reading.red, reading.green, reading.blue, reading.clear);
}

DetectedColor ColorSensor::classify(const ColorReading& reading) const {
	const float red = static_cast<float>(reading.red);
	const float green = static_cast<float>(reading.green);
	const float blue = static_cast<float>(reading.blue);
	const float largest = std::max(red, std::max(green, blue));
	const float smallest = std::min(red, std::min(green, blue));
	const float delta = largest - smallest;

	if (largest <= 0.0f || delta / largest < 0.2f) {
		return DetectedColor::None;
	}

	float hue;
	if (largest == red) {
		hue = 60.0f * ((green - blue) / delta);
		if (hue < 0.0f) {
			hue += 360.0f;
		}
	} else if (largest == green) {
		hue = 60.0f * ((blue - red) / delta + 2.0f);
	} else {
		hue = 60.0f * ((red - green) / delta + 4.0f);
	}

	if (hue < 15.0f || hue >= 345.0f) {
		return DetectedColor::Red;
	}
	if (hue >= 15.0f && hue < 65.0f) {
		return DetectedColor::Orange;
	}
	if (hue >= 65.0f && hue < 170.0f) {
		return DetectedColor::Green;
	}
	if (hue >= 235.0f && hue < 285.0f) {
		return DetectedColor::Indigo;
	}
	return DetectedColor::None;
}
