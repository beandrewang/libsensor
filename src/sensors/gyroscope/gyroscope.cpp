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

#include "gyroscope.h"

gyroscope::gyroscope(interface *intf) : 
					sensor(intf),
					x(0), y(0), z(0),
					xOffset(0), yOffset(0), zOffset(0),
					xVar(.0f), yVar(.0f), zVar(.0f),
					xScale(.0f), yScale(.0f), zScale(.0f),
					tA(.0f), tB(.0f)
{
}

int gyroscope::readData(float &X, float &Y, float &Z)
{
	int x = 0, y = 0, z = 0;
	float curTemp = .0f, X = .0f, Y = .0f, Z = .0f;
	assert(readRawData(x, y, z));
	assert(readTemperature(&curTemp));	

	int xDrift = curTemp * tA + tB;
	int yDrift = curTemp * tA + tB;
	int zDrift = curTemp * tA + tB;

	if(0 == tA && 0 == tB) // didn't do temperature drift experiment
	{
		xDrift = yDrift = zDrift = 0;
	}

	x = x - xDrift;
	y = y - yDrift;
	z = z - zDrift;
	
	X = (x - xOffset) * xScale;
	Y = (y - yOffset) * yScale;
	Z = (z - zOffset) * yScale;

	return 0;
}

int gyroscope::calibration()
{
	int x = 0, y = 0, z = 0, xx = 0, yy = 0, zz = 0;
	float curTemp = .0f;
	int xSum = 0, ySum = 0, zSum = 0, xxSum = 0, yySum = 0, zzSum = 0;

	for(int i = 0; i < NUMBER_SAMPLES_FOR_CALIBRATION; i++)
	{
		assert(!readRawData(x, y, z));
		assert(!readTemperature(&curTemp));	

		int xDrift = curTemp * tA + tB;
		int yDrift = curTemp * tA + tB;
		int zDrift = curTemp * tA + tB;

		if(0 == tA && 0 == tB) // didn't do temperature drift experiment
		{
			xDrift = yDrift = zDrift = 0;
		}

		x = x - xDrift;
		y = y - yDrift;
		z = z - zDrift;

		xx = x^2;
		yy = y^2;
		zz = z^2;

		xSum += x;
		ySum += y;
		zSum += z;

		xxSum += xx;
		yySum += yy;
		zzSum += zz;
	}

	xOffset = xSum / NUMBER_SAMPLES_FOR_CALIBRATION;
	yOffset = ySum / NUMBER_SAMPLES_FOR_CALIBRATION;
	zOffset = zSum / NUMBER_SAMPLES_FOR_CALIBRATION;

	xVar = (xxSum / NUMBER_SAMPLES_FOR_CALIBRATION - xOffset^2) * xScale^2;
	yVar = (yySum / NUMBER_SAMPLES_FOR_CALIBRATION - yOffset^2) * yScale^2;
	zVar = (zzSum / NUMBER_SAMPLES_FOR_CALIBRATION - zOffset^2) * zScale^2;

	return 0;
}

int gyroscope::readCalibrationParams(int &xoffset, int &yoffset, int&zoffset,
								     float &xvar, float &yvar, float &zvar)
{
	xoffset = xOffset;
	yoffset = yOffset;
	zoffset = zOffset;
	xvar = xVar;
	yvar = yVar;
	zvar = zVar;

	return 0;
}

int gyroscope::writeCalibrationParams(int xoffset, int yoffset, int zoffset,
								  	  float xvar, float yvar, float zvar)
{
	xOffset = xoffset;
	yOffset = yoffset;
	zOffset = zoffset;
	xVar = xvar;
	yVar = yvar;
	zVar = zvar;

	return 0;
}



