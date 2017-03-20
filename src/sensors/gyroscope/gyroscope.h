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

#ifndef __GYROSCOPE__H__
#define __GYROSCOPE__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sensor.h"
#include "interface.h"

class gyroscope : public sensor {
	public:
		gyroscope(interface *intf);
		~gyroscope() { }

		// read the physical data converted from the data in digital register
		// the default method is for 3-axis, you can overwrite this method if
		// you select 1-axis or 2-axis sensors
		// unit: rad/s
		int readData(float &X, float &Y, float &Z);

		// calculate the zero offset and variance,
		// you have to make the gyro be in stationary state before calling
		// this func
		int calibration();

		// read the offset, variants
		int readCalibrationParams(int &xoffset, int &yoffset, int&zoffset,
								  float &xvar, float &yvar, float &zvar);

		// set the calibration parameters from offline experiment.
		// you can calculate the offset and variant in Matlab by the raw
		// data samples.
		int writeCalibrationParams(int xoffset, int yoffset, int zoffset,
								  float xvar, float yvar, float zvar);

		// open these two method is to allow doing calibration offline.
		virtual int readRawData(int &X, int &Y, int &Z) {};
		virtual int readTemperature(float &temperature) {};

	private:
		// The zero offset, for the raw sensor data,
		// How to get the offset for each axis?
		// 1, Make the gyroscope in a stationary state, the output from each axis
		//	  should be 0.
		// 2, Read a seria of gyro raw data and calcaluate the mean value for each axis.
		// 3, Save the mean values, they are xOffset, yOffset, zOffset seperately.
		int xOffset, yOffset, zOffset;
		// The variance for physical data of the gyroscope,
		// it is used for kalman filter when doing sensor fusion with accelerometer
		float xVar, yVar, zVar;
		// The scales, which convert the raw data to physical
		// How to get the scale for each axis?
		// 1, Read the datasheet of this gyro and find a formular, which would
		// 	  convert the digital value from the register to rad/s.
		// 2, Use the formular to calculate the scale for each axis and save.
		float xScale, yScale, zScale;
		// the linear fitting for the temprature drift
		// the zero drift equation is: y = tA*x + tB
		// y, the drift
		// x, the cur temperature
		float tA, tB;
		static const int NUMBER_SAMPLES_FOR_CALIBRATION = 1000;
};


#ifdef __cplusplus
extern "C" }
#endif

#endif // __GYROSCOPE__H__


