#include "LoRa.h"

static const int CHANNEL = 0;
byte receivedbytes;
char message[256];

//  File ponter
FILE *file;
char filename[36];

// Clock time variables
time_t timer;
struct tm* tm_info;

int counter = 0;
char temp[50];

GPS GPS;

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
    // check board type for BOOST pin
    writeReg(RegPaConfig, (uint8_t)(0x80 | pw)); // Enable BOOST pin, Pout = 17-(15-pw) dBm

    if (pw == 20)
        writeReg(RegPaDac, readReg(RegPaDac) | 0x7); // writing 0x07 enables 20 dBm if pw = 15

    if (pw >= 17) {
        pw = 15;
    }
    else if (pw < 2) {
        pw = 2;
    }
    // Enable overcurrent function
    writeReg(0x0b,0x3b); // Overload current protection
}

PI_THREAD (myThread)
{
  while(1) {
	GPS.update();
  }
}

void LoRa::Setup() {
    wiringPiSetup();
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);
    wiringPiSPISetup(CHANNEL, 500000);
    SetupLoRa();
    configPower(power);
    ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
    ssd1306_clearDisplay();
    ssd1306_drawString((char*)"Ready");
    ssd1306_display();
    GPS.setup();
    piThreadCreate(myThread);
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

void updateDisplay(char *str) {
    ssd1306_clearDisplay();
    ssd1306_drawString(str);
    ssd1306_display();
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

    printf("Send: %s\n", frame);
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
                // Divide by 4
                SNR = (value & 0xFF) >> 2;
            //According to https://www.mouser.com/ds/2/761/down-767039.pdf
            rssicorr = 157;

            // Retrieve measurements
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            int packetrssi, rssi = 0;
            printf("%d:%d:%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
	    if(SNR>=0)
            	packetrssi = readReg(0x1A)-rssicorr;
	    else
		packetrssi = readReg(0x1A)-rssicorr+SNR*0.25;
            rssi = readReg(0x1B)-rssicorr;
            counter++;

            // Show measurements on small display
            char temp[301];
            sprintf(temp,"P. RSSI: %d\nRSSI: %d\nSNR: %li\nMsg: %s\nCounter: %d", packetrssi,rssi,SNR,message,counter);
            updateDisplay(temp);


            // Debug info
            printf("Packet RSSI: %d, ", packetrssi);
            printf("RSSI: %d, ", rssi);
            printf("SNR: %li, ", SNR);
            printf("Length: %i", (int)receivedbytes);
            printf("\n");
            printf("Payload: %s\n", message);
            printf("Counter: %d\n", counter);

	    //Update GPS data
	    //GPS.update();
	
            // Open csv file and save new data
            file = fopen(filename,"a");
            fprintf(file, "\n100;%d;%d;%li;%d;%d;%0.1lf;%0.1lf;'%s';%0.6lf;%0.6lf;%0.1lf",packetrssi,
                    rssi,SNR,power,sf,bw,freq,CR,GPS.getLat(),GPS.getLong(),GPS.getSpeed());
            fclose(file);
        }
    }
}

void LoRa::createCSVFile(string argv) {
    DIR* dir = opendir("csvfiles");
    if (dir)
        closedir(dir);
    else if (ENOENT == errno)
        mkdir("csvfiles",0777);
    time(&timer);
    tm_info = localtime(&timer);
    strftime(filename, sizeof(filename), "csvfiles/%Y-%m-%d_%H-%M-%S.csv", tm_info);
    file = fopen(filename, "w");
    if(argv=="receive")
        fprintf(file, "Distance [m];Packet RSSI [dBm];RSSI [dBm];SNR [dB];Power level [dBm];SF;BW [kHz];Freq [MHz];CR;Lat;Long;Speed [km/h]");
    else if(argv=="datarate")
        fprintf(file, "Distance [m];Data Rate [bps]");
    else if(argv=="rtt")
        fprintf(file, "Distance [m];Round trip time [ms]");
    fclose(file);
    printf("File created: %s\n", filename + strlen(filename) - 23);
}

void LoRa::getDatarate() {
    //Mask all IRQ beside valid header and rx done.
    writeReg(REG_IRQ_FLAGS_MASK, 0xAF);
    struct timespec start, end;
    char temp[50];
    int counter = 0;

    while(1) {
	// Wait for a valid header 
        while(readReg(REG_IRQ_FLAGS) == 0) {}
	// Start timer
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);
	// As long as the rx done flag isn't set do nothing
        while(readReg(REG_IRQ_FLAGS) == IRQ_LORA_HEADER) {}
	// Rx done flag is set, stop timer
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
	// Find time difference between start and stop
        uint64_t delta_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
        // Calculate bit rate as number of bytes received devided by the time it took
	double bitrate = (readReg(REG_RX_NB_BYTES)*8)/((double)delta_us/1000000);
        printf("Bit rate %d bps \n",(int)bitrate);
        counter++;

        // Save to file
        file = fopen(filename,"a");
        fprintf(file, "\n100;%d",(int)bitrate);
        fclose(file);

        // Show measurements on small display
        sprintf(temp,"Data rate: %d bps\nCounter: %d", (int)bitrate,counter);
        updateDisplay(temp);
        writeReg(REG_IRQ_FLAGS, 0xFF);
    }

}

void LoRa::getRtt() {
    byte ping[5] = "Ping";
    struct timespec start, end;
    while(1) {
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);
        txlora(ping, sizeof(ping));
        writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
        opmode(OPMODE_RX);
        while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {
            delay(1);
        }
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        int delta_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
        printf("Response: %d ms\n\n", delta_ms);

        // Save to file
        file = fopen(filename,"a");
        fprintf(file, "\n100;%d",delta_ms);
        fclose(file);


        delay(1000);
    }
}

int getFileSize() {
	char fileSizeMsg[10];
	//Mask all IRQ beside rx done
	writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
	while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {}
	if (receive(fileSizeMsg))
		cout << "Received file size" << endl;
		return stod(string(fileSizeMsg));
}

void LoRa::getFile() {
	struct timespec start, end;
	byte ack[4] = "Ack";
	byte nack[5] = "Nack";
	FILE *file2;

	bool firstRun = true;
	int fileSize = 339;	
	//int fileSize = getFileSize();
	//int numPackets = ceil((double)fileSize/255);
	int numPackets = 99;

	file2 = fopen("lenna2.jpg", "wb");
	clock_gettime(CLOCK_MONOTONIC_RAW, &start);
	for(int i = 0; i<numPackets; i++) 
	{
		opmode(OPMODE_RX);

		//Mask all IRQ beside rx done
		writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
			
		//Wait for receving packet
		while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {}
	
		if (receive(message)) {
			cout << "Received packet " << i+1 << " out of " << numPackets << endl;
			txlora(ack, sizeof(ack));
			fwrite(message, 1, receivedbytes, file2);	
		} 
		else
			txlora(nack, sizeof(nack));
	}
	clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        uint64_t delta_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
        double bitrate = (fileSize*8)/((double)delta_us/1000000);
	printf("Received an %d bytes file in %0.1f seconds\n", fileSize, delta_us/1000000.0);
        printf("Avg bit rate %d bps \n",(int)bitrate);

	fclose(file2);
}

void LoRa::getBER() {
	int numerrors = 0;
	int sequenceNum = 0;
	int lostpackets = 0;
	long int SNR;

	string str, strcomp;
	while(1) {
		if (digitalRead(dio0) == 1)
		{
        		memset(message, 0, 256);
        		if (receive(message)) 
			{
				str = message;
				if(str.substr(0,10).compare("HelloWorld") != 0) {
					cout << "Payload error" << endl;
					numerrors++;
					cout << "Number of errors: " << numerrors << endl;
				}
				else if(str.substr(10).compare(to_string(sequenceNum)) != 0) {
					lostpackets = stoi(str.substr(10))-sequenceNum;
					cout << "Lost " << lostpackets << " packet(s)" << endl;
					numerrors += lostpackets;
					cout << "Number of errors: " << numerrors << endl;
					sequenceNum = stoi(str.substr(10));
				}
        		} else {
				cout << "CRC error occurred" << endl;
				numerrors++;
				cout << "Number of errors: " << numerrors << endl;
			}
			sequenceNum++;
			
    		}	
		delay(1);
	}
}