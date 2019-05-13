#include "LoRa.h"
#include <stdlib.h>


using namespace std;

int main(int argc, char *argv[])
{
	if (argc < 2) {
		printf("Usage: argv[0] sender|rec [message]\n");
		exit(1);
	}
	byte hello[32] = "HELLO";
	LoRa LoRa(SF6, 868.1, 500, 20);
	LoRa.Setup();

	if (!strcmp("sender", argv[1])) {
		// enter standby mode (required for FIFO loading))
		//opmode(OPMODE_STANDBY);

		printf("Send packets at SF%i on %f Mhz.\n", LoRa.getSf(), LoRa.getFreq());
		printf("------------------\n");

		if (argc > 2)
			strncpy((char *)hello, argv[2], sizeof(hello));

		while (1) {
			LoRa.txlora(hello, strlen((char *)hello));
			delay(1000);
		}
	}
	else if(!strcmp("datarate", argv[1])) {
        	LoRa.opmode(OPMODE_RX);
		printf("Listening at SF%i on %.6lf Mhz.\n", LoRa.getSf(), LoRa.getFreq());
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
	else if(!strcmp("ber", argv[1])) {
		LoRa.opmode(OPMODE_RX);
		LoRa.getBER();
	}
	else {
		LoRa.opmode(OPMODE_RX);
		printf("Listening at SF%i on %.6lf Mhz.\n", LoRa.getSf(), LoRa.getFreq());
		
		LoRa.createCSVFile(argv[1]);
		printf("------------------\n");
		while (1) {
			LoRa.receivepacket();
			delay(1);
		}

	}

	return (0);
}
