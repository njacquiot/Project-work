#include "LoRa.h"
#include <string.h>
#include <stdlib.h>

using namespace std;

int main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("Error: unknown command, use oneone of the following arguments: sender, receive, rtt, filesend, per or command \n");
		exit(1);
	}
	// Create LoRa object
	LoRa LoRa(SF7, 868.1, 125, 20);
	// Run setup
	LoRa.Setup();
	//---------------------------------Different test modes--------------------------------------------------//
	if (!strcmp("sender", argv[1])) {
		if (argc < 3) {
			printf("The sender command should be followed with a message \n");
			exit(1);
		}
		LoRa.send(argv[2]);
	}
	else if(!strcmp("receive", argv[1])) {
		LoRa.opmode(OPMODE_RX);
		printf("Listening at SF%i on %.1f MHz.\n", LoRa.getSf(), LoRa.getFreq());
		printf("------------------\n");
		while (1) {
			LoRa.receivepacket();
			delay(1);
		}
	}
	else if(!strcmp("rtt", argv[1])) {
		LoRa.startRtt();
	}
	else if(!strcmp("filesend", argv[1])) {
		LoRa.startFileTransfer();
	}
	else if(!strcmp("per", argv[1])) {
		LoRa.startBERtest();
	}
	else if(!strcmp("command", argv[1])) {
		if (argc < 3) {
			printf("Error: please specify command along a parameter e.g. command takeoff,2\n");
			exit(1);
		}
		LoRa.sendCommand(argv[2]);
	}
	else {
		printf("Error: unknown command, use one of the following arguments: sender, receive, rtt, filesend, per or command \n");		
	}
	//-------------------------------------------------------------------------------------------------//
	return (0);
}
