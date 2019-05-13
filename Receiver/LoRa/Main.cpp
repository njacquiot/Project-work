#include "LoRa.h"
#include <stdlib.h>


using namespace std;

int main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("Error: unknown command, use oneone of the following arguments: sender, receive, datarate, rtt, filereceive, per or command \n");
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
		
		LoRa.createCSVFile(argv[1]);
		printf("------------------\n");
		while (1) {
			LoRa.receivepacket();
			delay(1);
		}
	}
	else if(!strcmp("datarate", argv[1])) {
        	LoRa.opmode(OPMODE_RX);
		printf("Listening at SF%i on %.1f MHz.\n", LoRa.getSf(), LoRa.getFreq());
		LoRa.createCSVFile(argv[1]);
		printf("------------------\n");
		LoRa.getDatarate();
	}
	else if(!strcmp("rtt", argv[1])) {
		LoRa.createCSVFile(argv[1]);
		LoRa.getRtt();
	}
	else if(!strcmp("filereceive", argv[1])) {
		LoRa.opmode(OPMODE_RX);
		LoRa.getFile();
	}
	else if(!strcmp("per", argv[1])) {
		LoRa.opmode(OPMODE_RX);
		LoRa.getPER();
	}
	else if(!strcmp("command", argv[1])) {
		LoRa.opmode(OPMODE_RX);
		LoRa.receivecommand();
	}
	else {
		printf("Error: unknown command, use one of the following arguments: sender, receive, datarate, rtt, filereceive, per or command \n");		
	}
	//-------------------------------------------------------------------------------------------------//
	return (0);
}
