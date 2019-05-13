#include "LoRa.h"
#include <stdlib.h>
#include <string.h>

static const int CHANNEL = 0;
byte receivedbytes;
char message[256];

// LoRa object
LoRa::LoRa(sf_t sf_, double freq_, double bw_, int power_)
{
	sf = sf_;
	freq = freq_;
	bw = bw_;
	power = power_;

}

// Select SPI receiver
void selectreceiver()
{
	digitalWrite(ssPin, LOW);
}
// Unselect SPI receiver
void unselectreceiver()
{
	digitalWrite(ssPin, HIGH);
}
// Read register
byte readReg(byte addr)
{
	unsigned char spibuf[2];

	selectreceiver();
	// MSB should be 0 to read from a register
	spibuf[0] = addr & 0x7F;
	spibuf[1] = 0x00;
	wiringPiSPIDataRW(CHANNEL, spibuf, 2);
	unselectreceiver();

	return spibuf[1];
}
// Write to register
void writeReg(byte addr, byte value)
{
	unsigned char spibuf[2];
	// MSB should be 1 to write to the register
	spibuf[0] = addr | 0x80;
	spibuf[1] = value;
	selectreceiver();
	wiringPiSPIDataRW(CHANNEL, spibuf, 2);

	unselectreceiver();
}
// Change the operating mode
void LoRa::opmode(uint8_t mode) {
	if (mode == OPMODE_LORA) {
		// LoRA mode enabled
		writeReg(REG_OPMODE, OPMODE_LORA);
	}
	else {
		writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);
	}
}
// Function that returns the bits that should be written into a register to get a specfic bandwidth
int getBwbits(double bw) {
	double availableBw[10] = { 7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500 };
	for (int i = 0; i < 10; i++)
	{
		if (availableBw[i] == bw)
			return i;	
	}
}
// Setup function that set the most important registers for LoRa communication
void LoRa::SetupLoRa()
{
	// Reset device
	digitalWrite(RST, LOW);
	delay(100);
	digitalWrite(RST, HIGH);
	delay(100);
	// Read chip version
	byte version = readReg(REG_VERSION);
	if (version == 0x12) {
		printf("SX1276 detected, starting.\n");
	}
	else {
		printf("Unrecognized transceiver.\n");
		exit(1);
	}
	// Default mode is FSK instead of LoRa
	opmode(OPMODE_LORA);

	// set frequency
	uint64_t frf = ((uint64_t)(freq * 1000000) << 19) / 32000000;           // Step size 2^19   32MHz clock 
	writeReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeReg(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeReg(REG_FRF_LSB, (uint8_t)(frf >> 0));

	if (sf == SF11 || sf == SF12) {
		// Enable automatic gain control + Low Data Rate Optimization (Mandated when the symbol length exceeds 16ms)
		writeReg(REG_MODEM_CONFIG3, 0x0C);   
	}
	else {
		// Enable automatic gain control
		writeReg(REG_MODEM_CONFIG3, 0x04);   
	}
	// Explicit header mode (20 bits: payload length, CR parameter, CRC presence ), CR: 4/5, BW = userdefined 
	writeReg(REG_MODEM_CONFIG, getBwbits(bw) << 4 | 0x02); 
	// Set spreading factor, CRC is on
	writeReg(REG_MODEM_CONFIG2, (sf << 4) | 0x04);

	if (sf == SF10 || sf == SF11 || sf == SF12) {
		// Symbol timeout: TimeOut = SymbTimeout * Ts
		writeReg(REG_SYMB_TIMEOUT_LSB, 0x05);    
	}
	else {
		// Symbol timeout: TimeOut = SymbTimeout * Ts
		writeReg(REG_SYMB_TIMEOUT_LSB, 0x08);    
	}
	// Max Payload length 255 bytes
	writeReg(REG_MAX_PAYLOAD_LENGTH, 0xFF);    
	// Payload length 255 bytes  
	writeReg(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);  
	// Symbol periods between frequency hops (enabled), 1st hop always happen after the 1st header symbol  
	writeReg(REG_HOP_PERIOD, 0xFF); 
	// Current value of RX databuffer pointer (address of last byte written by LoRa receiver)
	writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD)); 
	// LNA settings, HF: boost on, 150% LNA current, LF: default lna current, highest gain possible
	writeReg(REG_LNA, LNA_MAX_GAIN);    
	// set PA ramp-up time 50 uSec
	writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); 
}

static void configPower(int8_t pw) {
    if (pw == 20) {
        // writing 0x07 enables 20 dBm, default is 14 dBm
        writeReg(RegPaDac, readReg(RegPaDac) | 0x7); 
    }

    if (pw > 15) {
        // Output power is set with only 4 bits (different formulas to calcualte output power in data sheet)
        pw = 15;
    }
    else if (pw < 2) {
        // Lowest output power
        pw = 2;
    }
    // Enable BOOST pin, Pout = 17-(15-pw) dBm
    writeReg(RegPaConfig, (uint8_t)(0x80 | pw)); 
    // Increase overload current protection to 240 mA
    writeReg(REG_OCP,0x3b); 
}
// Setup function that itialize SPI, LoRa and output power
void LoRa::Setup() {
	wiringPiSetup();
	pinMode(ssPin, OUTPUT);
	pinMode(dio0, INPUT);
	pinMode(RST, OUTPUT);
	wiringPiSPISetup(CHANNEL, 500000);
	SetupLoRa();
	configPower(power);
}
// Get current frequency
double LoRa::getFreq() {
	return freq;
}
// Get current spreading factor
sf_t LoRa::getSf() {
	return sf;
}
// Funciton used to fill the fifobuffer when transmitting
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
	// Set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
	writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP); // Interrupt on DIO0 when tx is done, DIO1/2 not used
	// Clear all radio IRQ flags
	writeReg(REG_IRQ_FLAGS, 0xFF);
	// Mask all IRQs but TxDone
	writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

	// initialize the payload size and address pointers
	writeReg(REG_FIFO_TX_BASE_AD, 0x00);    // the point in memory where the transmit information is stored   
	writeReg(REG_FIFO_ADDR_PTR, 0x00);      // set fifo pointer to address 0x00
	writeReg(REG_PAYLOAD_LENGTH, datalen);  // indicates the size of the memory location to be transmitted

	// Write data to FIFO buffer
	writeBuf(REG_FIFO, frame, datalen);
	// Change opmode to start the transmission
	opmode(OPMODE_TX);

	//printf("Send: %s\n", frame);
}
// Receive function that read the FIFO buffer when a Rx done interrupt has occured
bool receive(char *payload) {
	// clear rxDone IRQ
	writeReg(REG_IRQ_FLAGS, 0x40);
	// Read IRQ flags
	int irqflags = readReg(REG_IRQ_FLAGS);

	//  Check whether a CRC error has occurred
	if ((irqflags & 0x20) == 0x20)
	{
		printf("CRC error\n");
		// Reset CRC flag
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
	// Check if dio0 goes high, which indicate a new packet has arrived.
	if (digitalRead(dio0) == 1)
	{	
		memset(message, 0, 256);
		if (receive(message)) {
			long int SNR;
			int rssicorr;
			
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
			//According to https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf
			rssicorr = 157;

			// Print debug info to console
			printf("Packet RSSI: %d, ", readReg(REG_PKT_RSSI_VALUE) - rssicorr); // RSSI of last packet
			printf("RSSI: %d, ", readReg(REG_RSSI_VALUE) - rssicorr); // Current RSSI
			printf("SNR: %li, ", SNR);
			printf("Length: %i\n", (int)receivedbytes);
			printf("Payload: %s\n", message);
		}
	}
}

// Start the rtt test mode that responds on a ping message
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
// Function that transfers a png image to the receiver
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
		// Determine file size
		fseek(file, 0 , SEEK_END);
  		long fileSize = ftell(file);
		fseek(file, 0, SEEK_SET);
		snprintf((char *)numBytes, sizeof(numBytes), "%ld", fileSize);
		int numPackets = ceil((double)fileSize/255);
		cout << "Sending file size" << endl;
		txlora(numBytes, sizeof(numBytes));
		// Wait for the transmission to complete
		while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_TXDONE) {}
		// Continue as long as data is available
		while ((bytesRead = fread(temp, 1, sizeof(temp), file)) > 0)
		{
			// Transmit chunk of data
			txlora(temp, bytesRead);
			cout << "Sent packet " << ++numsent << " out of " << numPackets << endl;
			while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_TXDONE) {}
			
			// Change mask to Rx done
			writeReg(REG_IRQ_FLAGS_MASK,~IRQ_LORA_RXDONE_MASK);
			// Change operating mode to receive
			opmode(OPMODE_RX);
			// Start timer 
			clock_gettime(CLOCK_MONOTONIC_RAW, &start);
			// Waiting to receive an acknowlegdment
			while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {
				clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        			uint64_t delta_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
				// Not a perfect solution, but in case of a missed ack, continue after waiting a certain amount of time 
				if(delta_us > 500000) {
					cout << "I'm breaking free" << endl;
					break;
				}
			}

			if (receive(message)) {
				if(string(message) == "Ack")
					cout << "Received: Ack" << endl;
				else if(string(message) == "Nack") {
					// Retransmit message in case the receiver has sent a Nack
					txlora(temp, bytesRead);
					cout << "Retranmission" << endl;
					// Wait for the transmission to complete
					while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_TXDONE) {}
				}	
			}
		}
		cout << "File transfer complete" << endl;
	}
	fclose(file);
}
// Start PER test (an eequivalent function should run on the receivr)
void LoRa::startPERtest() {
	int sequenceNum = 0;
	// Unique payload known at the receiver and is used to test if payload errors has occured
	string hello = "HelloWorld";
	byte temp[20];
	while(1) {
		// Add sequence number to payload
		strcpy((char*)temp, (hello + to_string(sequenceNum)).c_str());
		// Transmit packet
		txlora(temp, sizeof(temp));
		sequenceNum++;
		cout << sequenceNum << endl;
		// Delay depends on chosen SF and BW
		delay(100);
	}
	
}
// Send function
void LoRa::send(char * message) {
	byte temp[256];
	printf("Send packets at SF%i on %.1f MHz.\n", getSf(), getFreq());
	printf("------------------\n");
	strncpy((char *)temp, message, sizeof(temp));
	while (1) {
		txlora(temp, strlen((char *)temp));
		delay(1000);
	}
}
// Send control commands 
void LoRa::sendCommand(char * message) {
	byte temp[256];
	printf("Send command at SF%i on %.1f MHz.\n", getSf(), getFreq());
	printf("------------------\n");
	strncpy((char *)temp, message, sizeof(temp));
	txlora(temp, strlen((char *)temp));
}
