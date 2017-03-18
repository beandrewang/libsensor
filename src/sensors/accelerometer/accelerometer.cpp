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

#include <cmath.h>
#include "accelerometer.h"

accelerometer::accelerometer(interface *intf) :
 							  sensor(intf),
							  xOffset(0), yOffset(0), zOffset(0),
							  xVar(.0f), yVar(.0f), zVar(.0f),
							  xScale(.0f), yScale(.0f), zScale(.0f),
							  tA(.0f), tB(.0f),
							  raw0gThreshold(0), raw1g(0)
{
}

int accelerometer::readData(float &X, float &Y, float &Z)
{
	int x = 0, y = 0, z = 0;
	float curTemp = .0f, X = .0f, Y = .0f, Z = .0f;
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
	
	X = (x - xOffset) * xScale;
	Y = (y - yOffset) * yScale;
	Z = (z - zOffset) * yScale;

	return 0;
}

int accelerometer::calibration()
{
	bool b001 = false, b00_1 = false, b010 = false, b0_10 = false,
		 b100 = false, b_100 = false;

	int count001 = 0, count00_1 = 0, count010 = 0, count0_10 = 0,
		count100 = 0, count_100 = 0;

	int x = 0, y = 0, z = 0;
	int xSum[4] = {0}, ySum[4] = {0}, zSum[4] = [0];
	int xxSum[4] = {0}, yySum[4] = {0}, zzSum[4] = {0};
	int x1gSum = 0, y1gSum = 0, z1gSum = 0;
	
	while(~(b001 & b00_1 & b010 & b0_10 & b100 & b_100))
	{
		assert(!readRawData(x, y, z));
		
		if(std::abs(x) < raw0gThreshold &&
		   std::abs(y) < raw0gThreshold &&
		   (z - raw1g) < raw0gThreshold) 
		{
			// 001g
			if(count001++ < NUMBER_SAMPLES_FOR_CALIBRATION)
			{
				xSum[0] += x;
				ySum[0] += y;
				xxSum[0] += x^2;
				yySum[0] += y^2;
				z1gSum += z;
			}
			else
			{
				b001 = true;
			}
		}
		else if(std::abs(x) < raw0gThreshold &&
 			    std::abs(y) < raw0gThreshold &&
 			    (-z + raw1g) < raw0gThreshold) 
		{
			//00-1g
			if(count00_1++ < NUMBER_SAMPLES_FOR_CALIBRATION)
			{
				xSum[1] += x;
				ySum[1] += y; 
				xxSum[1] += x^2;
				yySum[1] += y^2;
			}
			else
			{
				b00_1 = true;
			}
		}
		else if(std::abs(x) < raw0gThreshold &&
				std::abs(z) < raw0gThreshold &&
				(y - raw1g) < raw0gThreshold)
		{
			// 01g0
			if(count010++ < NUMBER_SAMPLES_FOR_CALIBRATION)
			{
				xSum[2] += x;
				zSum[0] += z;
				xxSum[2] += x^2;
				zzSum[0] += z^2;
				y1gSum += y;
			}
			else
			{
				b010 = true;
			}
		}
		else if(std::abs(x) < raw0gThreshold &&
				std::abs(z) < raw0gThreshold &&
				(-y + raw1g) < raw0gThreshold)
		{
			// 0-1g0
			if(count0_10++ < NUMBER_SAMPLES_FOR_CALIBRATION)
			{
				xSum[3] += x;
				zSum[1] += z;
				xxSum[3] += x^2;
				zzSum[1] += z^2;
			}
			else
			{
				b0_10 = true;
			}
		}
		else if(std::abs(y) < raw0gThreshold &&
				std::abs(z) < raw0gThreshold &&
				(y - raw1g) < raw0gThreshold)
		{
			// 1g00
			if(count100++ < NUMBER_SAMPLES_FOR_CALIBRATION)
			{
				ySum[2] += y;
				zSum[2] += z;
				yySum[2] += y^2;
				zzSum[2] += z^2;
				x1gSum += x;
			}
			else
			{
				b100 = true;
			}
		}
		else if(std::abs(y) < raw0gThreshold &&
				std::abs(z) < raw0gThreshold &&
				(-y + raw1g) < raw0gThreshold)
		{
			// -1g00
			if(count_100++ < NUMBER_SAMPLES_FOR_CALIBRATION)
			{
				ySum[3] += y;
				zSum[3] += z;
				yySum[3] += y^2;
				zzSum[3] += z^2;
			}
			else
			{
				b_100 = true;
			}
		}
	}

	xOffset = (xSum[0] + xSum[1] + xSum[2] + xSum[3]) / NUMBER_SAMPLES_FOR_CALIBRATION
			  / 4;
	yOffset = (ySum[0] + ySum[1] + ySum[2] + ySum[3]) / NUMBER_SAMPLES_FOR_CALIBRATION
			  / 4;
	zOffset = (zSum[0] + zSum[1] + zSum[2] + zSum[3]) / NUMBER_SAMPLES_FOR_CALIBRATION
			  / 4;

	xScale = PHYSICAL_1G / (x1gSum / NUMBER_SAMPLES_FOR_CALIBRATION - xOffset);
	yScale = PHYSICAL_1G / (y1gSum / NUMBER_SAMPLES_FOR_CALIBRATION - yOffset);
	zScale = PHYSICAL_1G / (z1gSum / NUMBER_SAMPLES_FOR_CALIBRATION - zOffset);

	xVar = ((xxSum[0] + xxSum[1] + xxSum[2] + xxSum[3]) / NUMBER_SAMPLES_FOR_CALIBRATION
			/ 4 - xOffset^2) * xScale^2;
	yVar = ((yySum[0] + yySum[1] + yySum[2] + yySum[3]) / NUMBER_SAMPLES_FOR_CALIBRATION
			/ 4 - yOffset^2) * yScale^2;
	zVar = ((zzSum[0] + zzSum[1] + zzSum[2] + zzSum[3]) / NUMBER_SAMPLES_FOR_CALIBRATION
			/ 4 - zOffset^2) * zScale^2;

	return 0;
}

int accelerometer::readCalibrationParams(int &xoffset, int &yoffset, int&zoffset,
										 float &xscale, float &yscale, float zscale,
								  		 float &xvar, float &yvar, float &zvar)
{
	xoffset = xOffset;
	yoffset = yOffset;
	zoffset = zOffset;
	xscale = xScale;
	yscale = yScale;
	zscale = zScale;
	xvar = xVar;
	yvar = yVar;
	zvar = zVar;

	return 0;
}

int accelerometer::writeCalibrationParams(int xoffset, int yoffset, int zoffset,
										  float xscale, float yscale, float zscale,
										  float xvar, float yvar, float zvar)
{
	xOffset = xoffset;
	yOffset = yoffset;
	zOffset = zoffset;
	xScale = xscale;
	yScale = yscale;
	zScale = zscale;
	xVar = xvar;
	yVar = yvar;
	zVar = zvar;
	
	return 0;
}



