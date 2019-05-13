#include "gps.h"
string str;
string temp;
int fd;

// Constructor
GPS::GPS() {
    lat = 0;
    lon = 0;
    speed = 0;
}
// Get latitude
double GPS::getLat()
{
    return lat;
}
// Set latitude
void GPS::setLat(string latitude) {
    try {
    	//Original format is ddmm.mmmm, convert to decimal degrees
    	lat = stod(latitude.substr(0, 2)) + stod(latitude.substr(2, 7)) / 60;
    }
    // Catch in case the string can't be converted to a number
    catch(const std::invalid_argument&) {
	lat = 0;
    }
}

// Get longitude
double GPS::getLong() {
    return lon;
}
// Set longitude
void GPS::setLong(string longitude) {
    try {
    	//Original format is ddmm.mmmm, convert to decimal degrees
    	lon = stod(longitude.substr(0, 2)) + stod(longitude.substr(2, 7)) / 60;
    }
    // Catch in case the string can't be converted to a number
    catch(const std::invalid_argument&) {
	lon = 0;
    }
}

// Get ground speed
double GPS::getSpeed() {
    return speed;
}

// Set ground speed
void GPS::setSpeed(string knots) {
    try {
    	// Knots to km/h	
    	speed = stod(knots)*1.852;
    }
    // Catch in case the string can't be converted to a number
    catch(const std::invalid_argument&) {
    	speed = 0;
    }
}

// Open serial connection, default baud rate for GPS is 9600
void GPS::setup() {
    if ((fd = serialOpen ("/dev/ttyS0", 9600)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        exit(1);
    }

    if (wiringPiSetup () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        exit(1);
    }
}

// Update GPS data
void GPS::update() {   
    str.clear();
    temp.clear();
    // Wait for recommended minimum position data (RMC)
    while(str.compare("GPRMC") != 0) {
        str.clear();
        while(serialGetchar(fd) != '$') {}
        for(int i=0; i<5; i++) {
            str += (char)serialGetchar(fd);
        }
    }

    for(int j=0; j<=49; j++) {
  	temp += serialGetchar(fd);
    }
    //Check if data is valid (have a gps fix)
    if(temp[12] == 'A') {
    	setLat(temp.substr(14, 9));
    	setLong(temp.substr(27, 9));
    	setSpeed(temp.substr(39, 4));
    }
}
