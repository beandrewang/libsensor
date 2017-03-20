/*
	Copyright (c) 2017 Andrew Wang

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
 */

#ifndef __LIBSENSOR__ACCELEROMETER__H__
#define __LIBSENSOR__ACCELEROMETER__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor.h"
#include "interface.h"

namespace libsensor{
	class accelerometer;
}

class accelerometer : public sensor {
	public:
		accelerometer(interface *intf);
		~accelerometer();

		// read the physical data converted from the data in digital register.
		// the default method is for 3-axis, you can overwrite this method if
		// you select 1-axis or 2-axis sensors
		// unit: m/s^2
		int readData(float &X, float &Y, float &Z);

		// calculate the zero offset and variance,
		// you have to make the accelerometer be in 6 orientations,
		// -----------------------------------------------
		// X   0g  -1g   0g   +1g   0g   0g
		// Y  -1g   0g  +1g    0g   0g   0g
		// Z   0g   0g   0g    0g  +1g  -1g
		// -----------------------------------------------
		// before calling this func
		//
		// Anyway, you can directly set the calibration parameters if you
		// want to do the calibration offline. For example, you can collect
		// the samples and calculate the calibration params by Matlab or
		// other software, set the calibration parameters by calling function
		// - writeCalibrationParams
		int calibration();

		// read the offset, variants
		int readCalibrationParams(int &xoffset, int &yoffset, int&zoffset,
								  float &xscale, float &yscale, float zscale,
								  float &xvar, float &yvar, float &zvar);

		// if you don't want use the online calibration, you can set the related
		// parameters directly by calling this func
		int writeCalibrationParams(int xoffset, int yoffset, int zoffset,
								  float xscale, float yscale, float zscale,
								  float xvar, float yvar, float zvar);

		// open these two funcs is to allow doing calibration offline
		virtual int readRawData(int &X, int &Y, int &Z) {};
		virtual int readTemperature(float &temperature) {};
	private:
		// from the datasheet, different between chips
		int raw0gThreshold;
		int raw1g;
		// the zero offset of the raw data
		int xOffset, yOffset, zOffset;
		// the variant for the physical value, used for kalman filter for sensor fusion
		float xVar, yVar, zVar;
		// the scale, which converts the digital value from the register to physical
		// here we use 3 scales to map the digital value to physicals
        // but there is a more precise way to map, by a transform matrix
		// AX = B
		// A is the transform matrix
		// B is a identity matrix [1 0 0; 0 1 0; 0 0 1]
		// X is the samples average collected whey x = 1g, y = 1g, z = 1g.
		// This way, A = X^-1 * 9.8
		// if you want to use this more precise transform, rewrite the calibration
		// method.
		float xScale, yScale, zScale;
		// the linear fitting for the temprature drift
		// the zero drift equation is: y = tA*x + tB
		// y, the drift
		// x, the cur temperature
		float tA, tB;
		static const int NUMBER_SAMPLES_FOR_CALIBRATION = 1000;
		static const float PHYSICAL_1G = 9.80665f; // 9.8m/s^2
};

#ifdef __cplusplus
extern "C" }
#endif

#endif // __LIBSENSOR__ACCELEROMETER__H__

