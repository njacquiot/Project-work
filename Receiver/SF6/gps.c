#include "gps.h"
string str;
string temp;
int fd;

GPS::GPS() {
    lat = 0;
    lon = 0;
    speed = 0;
}

double GPS::getLat()
{
    return lat;
}

void GPS::setLat(string latitude) {
    try {
    	//Original format is ddmm.mmmm, convert to decimal degrees
    	lat = stod(latitude.substr(0, 2)) + stod(latitude.substr(2, 7)) / 60;
    }
    catch(const std::invalid_argument&) {
	lat = 0;
    }
}


double GPS::getLong() {
    return lon;
}

void GPS::setLong(string longitude) {
    try {
    	//Original format is ddmm.mmmm, convert to decimal degrees
    	lon = stod(longitude.substr(0, 2)) + stod(longitude.substr(2, 7)) / 60;
    }
    catch(const std::invalid_argument&) {
	lon = 0;
    }
}

double GPS::getSpeed() {
    return speed;
}

void GPS::setSpeed(string knots) {
    try {
    	// Knots to km/h	
    	speed = stod(knots)*1.852;
    }
    catch(const std::invalid_argument&) {
    	speed = 0;
    }
}

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

void GPS::update() {   
    str.clear();
    temp.clear();
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
    //Check if data is valid (have a gps fix) else use default values
    if(temp[12] == 'A') {
    	setLat(temp.substr(14, 9));
    	setLong(temp.substr(27, 9));
    	setSpeed(temp.substr(39, 4));
    }
}
