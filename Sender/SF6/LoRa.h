#pragma once
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <inttypes.h>
#include "LoRaRegisters.h"
#include <wiringPi.h>
#include <wiringPiSPI.h>

using namespace std;

typedef unsigned char byte;

enum sf_t { SF6 = 6, SF7, SF8, SF9, SF10, SF11, SF12 };

// Raspberry connections
static int ssPin = 6;
static int dio0 = 7;
static int RST = 0;

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
	void startRtt();
	void startFileTransfer();
	void startBERtest();
private:
	double freq;
	double bw;
	int power;
	sf_t sf;
};

