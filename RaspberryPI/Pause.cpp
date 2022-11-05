#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <wiringPi.h>

using namespace std;
using namespace cv;
using namespace raspicam;


int main()
{
	wiringPiSetup();
	
	pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
    
    while(1)
    {
		digitalWrite(21, 0);
		digitalWrite(22, 0);    //decimal = 9
		digitalWrite(23, 0);
		digitalWrite(24, 0);
	}
	return 0;
}
