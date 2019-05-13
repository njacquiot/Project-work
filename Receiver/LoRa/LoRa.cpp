#include "LoRa.h"
// The SPI channel
static const int CHANNEL = 0;
byte receivedbytes;
char message[256];

//  File ponter
FILE *file;
char filename[36];

// Clock time variables
time_t timer;
struct tm* tm_info;

// Incremented on the display and in the console every time a packet is received
int counter = 0;

GPS GPS;

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
// Determine the output power
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

// Separate thread to update GPS that only refresh each second
PI_THREAD (myThread)
{
  while(1) {
    GPS.update();
  }
}

// Setup function that itialize SPI, LoRa, output power, display and GPS
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
    // Indicate that the display is "Ready"
    ssd1306_display();
    GPS.setup();
    // Create seperate thread to update GPS data
    piThreadCreate(myThread);
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
    // MSB should be 1 to write
    spibuf[0] = addr | 0x80;
    for (int i = 0; i < len; i++) {
        spibuf[i + 1] = value[i];
    }
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);
    unselectreceiver();
}
// Clear display and draw a new string
void updateDisplay(char *str) {
    ssd1306_clearDisplay();
    ssd1306_drawString(str);
    ssd1306_display();
}

void LoRa::txlora(byte *frame, byte datalen) {
    // Set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP | MAP_DIO2_LORA_NOP); 
    // Clear all IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF);
    // Mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

    // Initialize the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00);    // the point in memory where the transmit information is stored
    writeReg(REG_FIFO_ADDR_PTR, 0x00);      // set fifo pointer to address 0x00
    writeReg(REG_PAYLOAD_LENGTH, datalen);  // indicates the size of the memory location to be transmitted

    // Write data to FIFO buffer
    writeBuf(REG_FIFO, frame, datalen);
    // Change opmode to start the transmission
    opmode(OPMODE_TX);

    printf("Send: %s\n", frame);
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
                // Divide by 4
                SNR = (value & 0xFF) >> 2;
            //According to https://www.semtech.com/uploads/documents/DS_SX1276-7-8-9_W_APP_V5.pdf
            rssicorr = 157;

            // Retrieve measurements
            time_t t = time(NULL);
            struct tm tm = *localtime(&t);
            int packetrssi, rssi = 0;
            printf("%d:%d:%d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
            if(SNR>=0)
                packetrssi = readReg(REG_PKT_RSSI_VALUE)-rssicorr;
            else
                packetrssi = readReg(REG_PKT_RSSI_VALUE)-rssicorr+SNR*0.25;
            rssi = readReg(REG_RSSI_VALUE)-rssicorr;
            counter++;

            // Show measurements on small display
            char temp[301];
            sprintf(temp,"P. RSSI: %d\nRSSI: %d\nSNR: %li\nMsg: %s\nCounter: %d", packetrssi,rssi,SNR,message,counter);
            updateDisplay(temp);


            // Print debug info to console
            printf("Packet RSSI: %d, ", packetrssi);
            //printf("RSSI: %d, ", rssi);
            //printf("SNR: %li, ", SNR);
            //printf("Length: %i", (int)receivedbytes);
            //printf("\n");
            //printf("Payload: %s\n", message);
            printf("Counter: %d\n", counter);
    
            // Open csv file and save new data
            file = fopen(filename,"a");
            fprintf(file, "\n100;%d;%d;%li;%d;%d;%0.1lf;%0.1lf;'%s';%0.6lf;%0.6lf;%0.1lf",packetrssi,
                    rssi,SNR,power,sf,bw,freq,CR,GPS.getLat(),GPS.getLong(),GPS.getSpeed());
            fclose(file);
        }
    }
}

// Function that create a CSV file, that depends on the chosen test mode
void LoRa::createCSVFile(string argv) {
    // Create csv files folder if no one exist
    DIR* dir = opendir("csvfiles");
    if (dir)
        closedir(dir);
    else if (ENOENT == errno)
        mkdir("csvfiles",0777);
    // Use date and time to name the csv file
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
// Function that measures the data rate and saves the value in a csv file
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
        // Clear all IRQ flags
        writeReg(REG_IRQ_FLAGS, 0xFF);
    }

}
// Function that measures the RTT each second and saves the value in a csv file (requires a sender to run an equivalent function)
void LoRa::getRtt() {
    byte ping[5] = "Ping";
    struct timespec start, end;
    while(1) {
        // Start the timer
        clock_gettime(CLOCK_MONOTONIC_RAW, &start);
        // Send ping message
        txlora(ping, sizeof(ping));
        // Mask all irq beside Rx done
        writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
        // Change operating mode to receive
        opmode(OPMODE_RX);
        // Wait for a response on the ping message
        while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {
            delay(1);
        }
        // Stop timer
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        // Calculate the RTT
        int delta_ms = (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
        printf("Response: %d ms\n\n", delta_ms);

        // Save to file
        file = fopen(filename,"a");
        fprintf(file, "\n100;%d",delta_ms);
        fclose(file);

        delay(1000);
    }
}

// Function to receive filesize which is sent in seperate packet before the actual file 
int getFileSize() {
    char fileSizeMsg[10];
    //Mask all IRQ beside Rx done
    writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
    // Wait for the packet
    while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {}
    if (receive(fileSizeMsg))
        cout << "Received file size" << endl;
        // Convert message to int value
        return stod(string(fileSizeMsg));
}

// Function that receives a file from the sender and prints the average data rate 
void LoRa::getFile() {
    struct timespec start, end;
    byte ack[4] = "Ack";
    byte nack[5] = "Nack";
    FILE *file2;

    bool firstRun = true;
    // Filesize is transmitted in a seperate packet
    int fileSize = getFileSize();
    // Determine the number of packets that should be received (payload size 255)
    int numPackets = ceil((double)fileSize/255);
    // The file is a png image
    file2 = fopen("lenna.png", "wb");
    // Start timer
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    for(int i = 0; i<numPackets; i++) 
    {
        // Change to receive mode
        opmode(OPMODE_RX);

        //Mask all IRQ beside Rx done
        writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_RXDONE_MASK);
            
        //Wait for receving packet
        while(readReg(REG_IRQ_FLAGS) != IRQ_LORA_RXDONE) {}
        
        if (receive(message)) {
            // Succesful reception
            cout << "Received packet " << i+1 << " out of " << numPackets << endl;
            txlora(ack, sizeof(ack));
            // Write received chunk to the png file
            fwrite(message, 1, receivedbytes, file2);   
        } 
        else
            // CRC error has occurred
            txlora(nack, sizeof(nack));
    }
    // File received, stop timer
    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    // Calculate the transmission time
    uint64_t delta_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
    // Find average data rate based on transmission time and file size
    double bitrate = (fileSize*8)/((double)delta_us/1000000);
    printf("Received an %d bytes file in %0.1f seconds\n", fileSize, delta_us/1000000.0);
        printf("Avg bit rate %d bps \n",(int)bitrate);

    fclose(file2);
}

// Start PER test (requires the sender to run an equivalent PER test function)
void LoRa::getPER() {
    int numerrors = 0;
    int sequenceNum = 0;
    int recvSeqNum = 0;
    int lostpackets = 0;
    long int SNR;

    string str, strcomp;
    while(1) {
        // Wait on new packet
        if (digitalRead(dio0) == 1)
        {
                memset(message, 0, 256);
                if (receive(message)) 
            {
                str = message;
                // Check if payload is correct
                if(str.substr(0,10).compare("HelloWorld") != 0) {
                    cout << "Payload error" << endl;
                    numerrors++;
                    cout << "Number of errors: " << numerrors << "Seq: " << sequenceNum << endl;
                }
                // Compare the received sequence number with the local counter
                else if(str.substr(10).compare(to_string(sequenceNum)) != 0) {
                    // In case any errors has occured in the received sequence number
                    try {
                            recvSeqNum = stoi(str.substr(10));
                        }
                        catch(const std::invalid_argument&) {
                        cout << "Payload error" << endl;
                        numerrors++;
                        recvSeqNum = sequenceNum;
                        }
                    lostpackets = recvSeqNum -sequenceNum;
                    if(lostpackets) {
                        cout << "Lost " << lostpackets << " packet(s)" << endl;
                        numerrors += lostpackets;
                        sequenceNum = recvSeqNum;
                    }
                    cout << "Number of errors: " << numerrors << "Seq: " << sequenceNum << endl;
                }
                } else {
                //CRC error occurred
                numerrors++;
                cout << "Number of errors: " << numerrors << "Seq: " << sequenceNum << endl;
            }
            sequenceNum++;
            }   
        delay(1);
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

// GPS needs to be disabled at setup, as it use the same serial port
void LoRa::receivecommand() {
    string str, command, argument;
    FILE * pythonfile;
    char * argv[2];
    int argc;
    // Wait for command to arrive
    while(digitalRead(dio0) != 1) {
        delay(1);
    }
    if (receive(message)) {
        int i = strcspn(message, ",");
        str = message;
        // Extract command froms string
        command = str.substr(0,i);
        if(command.compare("takeoff") == 0) {
            // Check whether required argument is given
            if(str.size() >= i+2) {
                argument = str.substr(i+1);
                cout << "Received command with argument:"<< argument << " meters" << endl;
            } 
            else {  
                cout << "No argument used, use default (2 meters)" << endl;
                argument = "2";
            }
            argc = 2;
            argv[0] = (char*)"takeoff.py";
            // String to char* array conversion
            argv[1] = &argument[0u];
            // Intialize python environment
            Py_Initialize();
            PySys_SetArgv(argc, argv);
            pythonfile = fopen("takeoff.py","r");
                // Run python file with argument
                PyRun_SimpleFile(pythonfile, "takeoff.py");
                Py_Finalize();
            cout << "Execute script" << endl;
        }
    }
}