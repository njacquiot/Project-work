#include "LoRa.h"
#include <stdlib.h>
#include <string.h>

static const int CHANNEL = 0;
byte receivedbytes;
char message[256];

LoRa::LoRa(sf_t sf_, double freq_, double bw_, int power_)
{
	sf = sf_;
	freq = freq_;
	bw = bw_;
	power = power_;

}

LoRa::~LoRa()
{
}

void selectreceiver()
{
	digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
	digitalWrite(ssPin, HIGH);
}

byte readReg(byte addr)
{
	unsigned char spibuf[2];

	selectreceiver();
	spibuf[0] = addr & 0x7F;
	spibuf[1] = 0x00;
	wiringPiSPIDataRW(CHANNEL, spibuf, 2);
	unselectreceiver();

	return spibuf[1];
}

void writeReg(byte addr, byte value)
{
	unsigned char spibuf[2];

	spibuf[0] = addr | 0x80;
	spibuf[1] = value;
	selectreceiver();
	wiringPiSPIDataRW(CHANNEL, spibuf, 2);

	unselectreceiver();
}

void LoRa::opmode(uint8_t mode) {
	if (mode == OPMODE_LORA) {
		writeReg(REG_OPMODE, OPMODE_LORA); // LoRA mode enabled
	}
	else {
		writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);
	}
}

int getBwbits(double bw) {
	double availableBw[10] = { 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500 };
	for (int i = 0; i < 10; i++)
	{
		if (availableBw[i] == bw)
			return i;	
	}
}


void LoRa::SetupLoRa()
{
	digitalWrite(RST, LOW);
	delay(100);
	digitalWrite(RST, HIGH);
	delay(100);
	byte version = readReg(REG_VERSION);
	
	if (version == 0x12) {
		printf("SX1276 detected, starting.\n");
	}
	else {
		printf("Unrecognized transceiver.\n");
		exit(1);
	}

	opmode(OPMODE_LORA);

	// set frequency
	uint64_t frf = ((uint64_t)(freq * 1000000) << 19) / 32000000;           // Step size 2^19   32MHz clock 
	writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));

	if (sf == SF11 || sf == SF12) {
		writeReg(REG_MODEM_CONFIG3, 0x0C);   // Enable automatic gain control + Low Data Rate Optimization (Mandated when the symbol length exceeds 16ms)
	}
	else {
		writeReg(REG_MODEM_CONFIG3, 0x04);   // Enable automatic gain control
	}
	writeReg(REG_MODEM_CONFIG, getBwbits(bw) << 4 | 0x03); // Explicit header mode (20 bits: payload length, CR parameter, CRC presence ), CR: 4/5, BW = userdefined 
	writeReg(REG_MODEM_CONFIG2, (sf << 4) | 0x04); // Set spreading factor, CRC on 

	if (sf == SF10 || sf == SF11 || sf == SF12) {
		writeReg(REG_SYMB_TIMEOUT_LSB, 0x05);    // Symbol timeout: TimeOut = SymbTimeout * Ts
	}
	else {
		writeReg(REG_SYMB_TIMEOUT_LSB, 0x08);    // Symbol timeout: TimeOut = SymbTimeout * Ts
	}
	
	writeReg(REG_DETECT_OPTIMIZE, 5);
	writeReg(REG_DETECTION_THRESHOLD, 0x0C);
	writeReg(REG_MAX_PAYLOAD_LENGTH, 0xFF);      // Max Payload length 255 bytes
	writeReg(REG_PAYLOAD_LENGTH, 0xFF);    // Payload length 255 bytes
	writeReg(REG_HOP_PERIOD, 0xFF); // Symbol periods between frequency hops (enabled), 1st hop always happen after the 1st header symbol
	writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD)); // Current value of RX databuffer pointer (address of last byte written by LoRa receiver)
	// read base address in FIFO data buffer for RX demodulator
	writeReg(REG_LNA, LNA_MAX_GAIN);    // LNA settings, HF: boost on, 150% LNA current, LF: default lna current, highest gain possible
	writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
}

static void configPower(int8_t pw) {
	if (pw == 20)
		writeReg(RegPaDac, readReg(RegPaDac) | 0x7); // Default value (17 dbm) instead of 20 dBm <-- 0x07 enable 20 dBm if pw = 15 (see below)

	if (pw >= 17) {
		pw = 15;
	}
	else if (pw < 2) {
		pw = 2;
	}
	// check board type for BOOST pin
	writeReg(RegPaConfig, (uint8_t)(0x80 | pw)); // Enable BOOST pin, Pout = 17-(15-pw) dBm
	// Enable overcurrent function
}

void LoRa::Setup() {
	wiringPiSetup();
	pinMode(ssPin, OUTPUT);
	pinMode(dio0, INPUT);
	pinMode(RST, OUTPUT);
	wiringPiSPISetup(CHANNEL, 500000);
	SetupLoRa();
	configPower(power);
}

double LoRa::getFreq() {
	return freq;
}

sf_t LoRa::getSf() {
	return sf;
}

static void writeBuf(byte addr, byte *value, byte len) {
	unsigned char spibuf[256];
	spibuf[0] = addr | 0x80;
	for (int i = 0; i < len; i++) {
		spibuf[i + 1] = value[i];
	}
	selectreceiver();
	wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);
	unselectreceiver();
}

void LoRa::txlora(byte *frame, byte datalen) {
	// set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
	writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP); // Interrupt on DIO0 when tx is done, DIO1/2 not used
	// clear all radio IRQ flags
	writeReg(REG_IRQ_FLAGS, 0xFF);
	// mask all IRQs but TxDone
	writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

	// initialize the payload size and address pointers
	writeReg(REG_FIFO_TX_BASE_AD, 0x00);    // the point in memory where the transmit information is stored   
	writeReg(REG_FIFO_ADDR_PTR, 0x00);      // set fifo pointer to address 0x00
	writeReg(REG_PAYLOAD_LENGTH, datalen);  // indicates the size of the memory location to be transmitted

	// download buffer to the radio FIFO
	writeBuf(REG_FIFO, frame, datalen);
	// now we actually start the transmission
	opmode(OPMODE_TX);

	//printf("Send: %s\n", frame);
}

bool receive(char *payload) {
	// clear rxDone IRQ
	writeReg(REG_IRQ_FLAGS, 0x40);
	// Read IRQ flags
	int irqflags = readReg(REG_IRQ_FLAGS);

	//  Check whether a CRC error has occurred
	if ((irqflags & 0x20) == 0x20)
	{
		printf("CRC error\n");
		// Reset flag
		writeReg(REG_IRQ_FLAGS, 0x20);
		return false;
	}
	else {
		// Read the location of the last packet received in the FIFO so that the last packet received can be easily read by pointing the register RegFifoAddrPtr to this register
		byte currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR);
		// Read the size of the memory location to be written in the event of a successful receive operation
		byte receivedCount = readReg(REG_RX_NB_BYTES);
		receivedbytes = receivedCount;
		// Move address pointer to current fifo address
		writeReg(REG_FIFO_ADDR_PTR, currentAddr);

		for (int i = 0; i < receivedCount; i++)
		{
			payload[i] = (char)readReg(REG_FIFO);
		}
	}
	return true;
}

void LoRa::receivepacket() {
	long int SNR;
	int rssicorr;

	if (digitalRead(dio0) == 1)
	{	
		memset(message, 0, 256);
		if (receive(message)) {
			// Estimation of SNR on last packet received (In two’s compliment format mutiplied by 4)
			byte value = readReg(REG_PKT_SNR_VALUE);
			if (value & 0x80) // The SNR sign bit is 1
			{
				// Invert and divide by 4
				value = ((~value + 1) & 0xFF) >> 2; // Shifting right by n bits on a two's complement signed binary number has the effect of dividing it by 2^n
				SNR = -value;
			}
			else
			{
				// Divide by 4
				SNR = (value & 0xFF) >> 2;
			}
			//According to https://www.mouser.com/ds/2/761/down-767039.pdf
			rssicorr = 157;
			printf("Packet RSSI: %d, ", readReg(REG_PKT_RSSI_VALUE) - rssicorr); // RSSI of last packet
			printf("RSSI: %d, ", readReg(REG_RSSI_VALUE) - rssicorr); // Current RSSI
			printf("SNR: %li, ", SNR);
			printf("Length: %i\n", (int)receivedbytes);
			printf("Payload: %s\n", message);
		}
	}
}

void LoRa::startRtt() {
	byte msg[4] = "Ack";
	while(1) {
	// mask all IRQs but RxDone
	writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
	opmode(OPMODE_RX);
	printf("Waiting to receive ping\n");
	while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {delay(1);}
	printf("Ping received, send ack\n");
	txlora(msg, sizeof(msg));
	}
}

void LoRa::startFileTransfer() {
	FILE *file;
	struct timespec start, end;
	size_t bytesRead = 0;
	byte numBytes[10];
	byte temp[255];
	int numsent = 0;
	file = fopen("lenna.png", "rb");
	if (file != NULL)
	{
		fseek(file, 0 , SEEK_END);
  		long fileSize = ftell(file);
		fseek(file, 0, SEEK_SET);
		snprintf((char *)numBytes, sizeof(numBytes), "%ld", fileSize);
		int numPackets = ceil((double)fileSize/255);
		cout << "Sending file size" << endl;
		//txlora(numBytes, sizeof(numBytes));
		//while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_TXDONE) {}
		
		while ((bytesRead = fread(temp, 1, sizeof(temp), file)) > 0)
		{
			txlora(temp, bytesRead);
			cout << "Sent packet " << ++numsent << " out of " << numPackets << endl;
			while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_TXDONE) {}
			
			// Change mask to rx done
			writeReg(REG_IRQ_FLAGS_MASK,~IRQ_LORA_RXDONE_MASK);
			opmode(OPMODE_RX);
			clock_gettime(CLOCK_MONOTONIC_RAW, &start);
			while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {
				clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        			uint64_t delta_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
				// Not a perfect solution, but in case of a missed ack, continue after waiting an amount of time 
				if(delta_us > 500000) {
					cout << "Im breaking free" << endl;
					break;
				}
			}

			if (receive(message)) {
				if(string(message) == "Ack")
					cout << "Received: Ack" << endl;
				else if(string(message) == "Nack") {
					txlora(temp, bytesRead);
					cout << "Retranmission" << endl;
					while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_TXDONE) {}
				}	
			}
		}
		cout << "File transfer complete" << endl;
	}
	fclose(file);
}

void LoRa::startBERtest() {
	int sequenceNum = 0;
	string hello = "HelloWorld";
	byte temp[20];
	while(1) {
		strcpy((char*)temp, (hello + to_string(sequenceNum)).c_str());
		txlora(temp, sizeof(temp));
		sequenceNum++;
		cout << sequenceNum << endl;
		delay(400);
	}
	
}
