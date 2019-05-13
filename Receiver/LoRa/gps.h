#pragma once
#include <wiringPi.h>
#include <wiringSerial.h>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <string>
#include <errno.h>

using namespace std;

class GPS
{
public:
	GPS();
	void setup();
	void update();
	void setLat(string lat);
	double getLat();
	void setLong(string lon);
	double getLong();
	void setSpeed(string knots);
	double getSpeed();
private:
	double lat;
	double lon;
	double speed;
};
