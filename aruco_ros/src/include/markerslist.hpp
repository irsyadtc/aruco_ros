#ifndef MARKERSLIST_HPP_
#define MARKERSLIST_HPP_

#include <cmath>

#define NO_OF_MARKERS 10


class markerslist
{
	
private:
    struct markersdb
	{
		std::string name;
		double posx, posy, posz;
		double roll, pitch, yaw;
	};
    
    markersdb MarkersDb[NO_OF_MARKERS] {
	{"marker_1", 13.3, 11.1, 1, M_PI_2, M_PI, 0},
	{"marker_2", 13.6, 8.9, 1, -1*M_PI_2, 0, M_PI_2},
	{"marker_3", 13.9, 6.64, 1, -1*M_PI_2, 0, M_PI_2},
	{"marker_4", 13.9, 5.04, 1, -1*M_PI_2, 0, M_PI_2},
	{"marker_5", 13.9, 1.95, 1, -1*M_PI_2, 0, M_PI_2},
	{"marker_6", 9.3, 0.19, 1, -1*M_PI_2, 0, 0},
	{"marker_7", 7.04, 0.153, 1, -1*M_PI_2, 0, 0},
	{"marker_8", 4.66, 0.19, 1, -1*M_PI_2, 0, 0},
	{"marker_9", 0.851, 1.07, 1, -1*M_PI_2, 0, -1*M_PI_2},
	{"marker_10", 0.248, 3.81, 1, -1*M_PI_2, 0, -1*M_PI_2}
	};
	

public:
	markerslist();
	~markerslist();

	unsigned int getSize() { return NO_OF_MARKERS; }
	std::string getName(int a) { return this->MarkersDb[a].name; }
	double getPosx(unsigned int a) { return this->MarkersDb[a].posx; }
	double getPosy(unsigned int a)  { return this->MarkersDb[a].posy; }
	double getPosz(unsigned int a)  { return this->MarkersDb[a].posz; }
	double getRoll(unsigned int a)  { return this->MarkersDb[a].roll; }
	double getPitch(unsigned int a)  { return this->MarkersDb[a].pitch; }
	double getYaw(unsigned int a)  { return this->MarkersDb[a].yaw; }
	
};



#endif
