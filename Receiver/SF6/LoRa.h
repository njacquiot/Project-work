#pragma once
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <thread>
#include <math.h>
#include "LoRaRegisters.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "ssd1306_i2c.h"
#include "gps.h"

using namespace std;

typedef unsigned char byte;

enum sf_t { SF6 = 6, SF7, SF8, SF9, SF10, SF11, SF12 };

// Raspberry connections
static int ssPin = 6;
static int dio0 = 7;
static int RST = 0;

#define CR  "4/5"
#define Distance 100

class LoRa
{
public:
	LoRa(sf_t sf, double freq, double bw, int power);
	~LoRa();
	void Setup();
	void txlora(byte *frame, byte datalen);
	void receivepacket();
	void SetupLoRa();
	double getFreq();
	sf_t getSf();
	void opmode(uint8_t mode);
	void createCSVFile(string argv);
	void getDatarate();
	void getRtt();
	void getFile();
	void getBER();
private:
	double freq;
	double bw;
	int power;
	sf_t sf;
};

