#include "../include/colorSensor.hpp"
#include <Arduino_APDS9960.h>

bool ColorSensor::begin() {
	return APDS.begin();
}

bool ColorSensor::read(ColorReading& reading) {
	if (!APDS.colorAvailable()) {
		return false;
	}

	return APDS.readColor(reading.red, reading.green, reading.blue, reading.clear);
}
