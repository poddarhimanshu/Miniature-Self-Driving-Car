#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <wiringPi.h>
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std;
using namespace cv;
using namespace raspicam;

/***************************************************************************************************************
frameCenter and source value may change for different camera positions, lane structure, size and distance.
inRange and Canny threshold values may change for different lighting conditions.

laneEnd tells us if the lane end is reached by calculating the intensity values in the frame, if at some point 
we start getting values much higher comparatively to black area of road that would mean a white strip of horizontal
lane end marker was found and then we send signal to Arduino device to take a UTurn.

****************************************************************************************************************/

// Image Processing Variables
Mat frame, matrix, framePers, frameGray, frameThresh, frameEdge, frameFinal, frameFinalDuplicate,frameFinalDuplicate1;
Mat roiLane, roiLaneEnd;
int leftLanePos, rightLanePos, frameCenter, laneCenter, result, laneEnd;

RaspiCam_Cv camera;

stringstream ss;


vector<int> histrogramLane, histrogramLaneEnd;

// Used hit and trial approach to get perfect 4 points on the frame such that we capture our lanes with some space left on the left and right sides
Point2f source[] = {Point2f(95,130),Point2f(360,130),Point2f(65,180), Point2f(390,180)};
// destination is where you want the source point to be perspective mapped onto the window, if you don't start with x = 0 for P1 and don't end at x = width for P2 then it takes the area from the left and right of the source image by that many pixels which you left on left and right 
Point2f Destination[] = {Point2f(100,0),Point2f(280,0),Point2f(100,240), Point2f(280,240)};

// Machine Learning Variables
CascadeClassifier StopCascade, ObjectCascade, TrafficCascade;
Mat frameStop, roiStop, grayStop, frameObject, roiObject, grayObject, frameTraffic, roiTraffic, grayTraffic;
vector<Rect> stop, object, traffic;

// will contain distance of detected objects : stop, obstacle, traffic
int distStop, distObject, distTraffic;

void Setup ( int argc,char **argv, RaspiCam_Cv &camera )
{
    camera.set ( CAP_PROP_FRAME_WIDTH,  ( "-w",argc,argv,400 ) );
    camera.set ( CAP_PROP_FRAME_HEIGHT,  ( "-h",argc,argv,240 ) );
    camera.set ( CAP_PROP_BRIGHTNESS, ( "-br",argc,argv,50 ) );
    camera.set ( CAP_PROP_CONTRAST ,( "-co",argc,argv,50 ) );
    camera.set ( CAP_PROP_SATURATION,  ( "-sa",argc,argv,50 ) );
    camera.set ( CAP_PROP_GAIN,  ( "-g",argc,argv ,50 ) );
    camera.set ( CAP_PROP_FPS,  ( "-fps",argc,argv,0));

}

void Capture()
{
	camera.grab();  // Captures the frame 
    camera.retrieve( frame); // retreives the frame in frame variable
	cvtColor(frame, frameStop, COLOR_BGR2RGB);
	cvtColor(frame, frameObject, COLOR_BGR2RGB); // By default the frame is in BGR color space, we need t convert it to RGB color space
	cvtColor(frame, frameTraffic, COLOR_BGR2RGB);
    cvtColor(frame, frame, COLOR_BGR2RGB);
     
}

void Perspective()
{
	// Create region of interest using the source points 
	line(frame,source[0], source[1], Scalar(0,0,255), 2);
	line(frame,source[1], source[3], Scalar(0,0,255), 2);
	line(frame,source[3], source[2], Scalar(0,0,255), 2);
	line(frame,source[2], source[0], Scalar(0,0,255), 2);
	
	// get the bird eye view of the track by doing a perspective transformatiion, result is stored in framePers
	matrix = getPerspectiveTransform(source, Destination);
	warpPerspective(frame, framePers, matrix, Size(400,240));
}

void Threshold()
{
	// convert frame from RGB space to Gray scale space for thresholding the white lane lines 
	cvtColor(framePers, frameGray, COLOR_RGB2GRAY);
	// using hit and trial method it was found that intensity of my while lanes lied in between 230 and 235 in the frame.
	inRange(frameGray, 230, 255, frameThresh);
	// get the edges in the image as well
	Canny(frameGray,frameEdge, 700, 650, 3, false);
	// rather than using frameThres for the calculations, we will be suing sum of frameEdge and frameGray
	add(frameThresh, frameEdge, frameFinal);
	cvtColor(frameFinal, frameFinal, COLOR_GRAY2RGB);
	//COLOR_GRAY2BGR color mode basically replaces all B, G, R channels with the gray value Y, so B=Y, G=Y, R=Y. It converts a single channel image to multichannel by replicating.
	cvtColor(frameFinal, frameFinalDuplicate, COLOR_RGB2BGR);   //used in histrogram function only
	cvtColor(frameFinal, frameFinalDuplicate1, COLOR_RGB2BGR);  
	
}

void Histrogram()
{
	// width of frame which will store sum of intensity values of each strip pof size : 1pixel
    histrogramLane.resize(400);
    histrogramLane.clear();
    
	// loop through each strip of pixel size 1
    for(int i=0; i<400; i++)       //frame.size().width = 400
    {
		// We get the region of interest by cropping out rectangle of height from 140-y pixel until 140+100pixels 
		// We do this to figure out the maximum intensity values in our image from start to end point. The position of that max value can be used to say that it contains a lane at that position
		roiLane = frameFinalDuplicate(Rect(i,140,1,100));
		divide(255, roiLane, roiLane); // normalize the values 
		// ROILane is a matrix of 3 color channels, height 100 and width 1, when we apply sum function to it, the sum function takes summation of entire matrix for 1*140 for all color channels and we take the first color channel, since all color channel are anyway replicated. So for each matrix of 1*100*3 we will have one scalr sum value which we are summing up to get the intensity value of that sector. 
		histrogramLane.push_back((int)(sum(roiLane)[0])); // 0 index to get summation of first color channel
    }
    
	// For each of our frame, we will also be checking if its lane end
    histrogramLaneEnd.resize(400);
    histrogramLaneEnd.clear();
	
	// process entire frame and loop through each strip of pixel size 1
    for (int i = 0; i < 400; i++)       
    {
	    roiLaneEnd = frameFinalDuplicate1(Rect(i, 0, 1, 240));   
	    divide(255, roiLaneEnd, roiLaneEnd);       
	    histrogramLaneEnd.push_back((int)(sum(roiLaneEnd)[0]));  
	    
    
    }
	
	// the value of laneEnd would be highest when the white horizontal strip is encounted.
    laneEnd = sum(histrogramLaneEnd)[0];
}

void LaneFinder()
{
    vector<int>:: iterator leftPtr;
	// Scan from point0 to point 150, could be 200 but we don't need to go as far until 200, as left lane will anyway be on the initial left side 
    leftPtr = max_element(histrogramLane.begin(), histrogramLane.begin() + 150);
	// leftLanePos stores actual postion of our first lane wrt to the left hand side of our frame 
    leftLanePos = distance(histrogramLane.begin(), leftPtr); 
    
    vector<int>:: iterator rightPtr;
	// Scan from point250 to point 400, could be 200 but we don't need to start as far as from 200 as right lane will anyway be on the utmost right 
    rightPtr = max_element(histrogramLane.begin() +250, histrogramLane.end());
	// rightLanePos stores actual postion of our second lane wrt to the left hand side of our frame 
    rightLanePos = distance(histrogramLane.begin(), rightPtr);
    
	// create a green line on the fram where we have identified our lanes. Show frameFinal to verify if all our calculations were correct and 
	// our green line is indeed coinciding with the lanes.
    line(frameFinal, Point2f(leftLanePos, 0), Point2f(leftLanePos, 240), Scalar(0, 255,0), 2);
    line(frameFinal, Point2f(rightLanePos, 0), Point2f(rightLanePos, 240), Scalar(0,255,0), 2); 
}

void LaneCenter()
{
	// stores the center of the white lanes 
    laneCenter = (rightLanePos-leftLanePos)/2 +leftLanePos;
	
	// ideally this frame center should be 200, but even after placing our car at the center of our lanes frame center and lane center were still 
	// not coinciding. This could be due to the slightest error in creating our ROI on the road for the lanes identification. Now its not possible to 
	// get exact ROI of lanes on the road but we can indeed shift our frame center to a little left, hence the value 188
    frameCenter = 188;
    
	// create a green line for the lane center
    line(frameFinal, Point2f(laneCenter,0), Point2f(laneCenter,240), Scalar(0,255,0), 3);
	// create a blue line for the frame center (BGR color space)
    line(frameFinal, Point2f(frameCenter,0), Point2f(frameCenter,240), Scalar(255,0,0), 3);
	
	// this would tell us by how much we need to move left, right or if we need to move forward. 
    result = laneCenter-frameCenter;
}

void StopDetection()
{
    // since our car is always moving forward on the right side of the road, the signals are also going to be 
	// present on the right side, hence we don't need the left side of the frame, so get rectangle (200,0,400,240)
    roiStop = frameStop(Rect(200,0,200,140));
	// convert the image to grayscale
    cvtColor(roiStop, grayStop, COLOR_RGB2GRAY);
	// equualize the frame to improve the contrast in an image, in order to stretch out the intensity range
    equalizeHist(grayStop, grayStop);
	// pass the input to cascade classifer : result is stored in stop.
    StopCascade.detectMultiScale(grayStop, stop);
    
	// An image may have multiple objects that was detected by the MultiScale function, hence we iterate over all the result objects
    for(int i=0; i<stop.size(); i++)
    {
		Point P1(stop[i].x, stop[i].y); // get the top left corner point in the detected object
		Point P2(stop[i].x + stop[i].width, stop[i].y + stop[i].height); // get the bottom right corner point in the detected object
	
		// create a rectangle using these 2 points
		rectangle(roiStop, P1, P2, Scalar(0, 0, 255), 2);
		// Display the text on screen
		putText(roiStop, "Stop Sign", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
		
		// contains the distance of object from camera, this value of m and c are calculate using linear equation, see pdf for more details
		distStop = (-0.62)*(P2.x-P1.x) + 83.65;
		
		// Show the distance of stop signal from camera
		ss.str(" ");
		ss.clear();
		ss<<"D = "<<distStop<<" cm";
		putText(roiStop, ss.str(), Point2f(1,130), 0,1, Scalar(0,0,255), 2);
    }
}

void ObjectDetection()
{
    // since object will only be present at the front of our camera, get the region (100,0,300, 180)
    roiObject = frameObject(Rect(100,0,200,180));
    cvtColor(roiObject, grayObject, COLOR_RGB2GRAY);
    equalizeHist(grayObject, grayObject);
    ObjectCascade.detectMultiScale(grayObject, object);
    
	// loop through all detected objects in that frame
    for(int i=0; i<object.size(); i++)
    {
		Point P1(object[i].x, object[i].y);
		Point P2(object[i].x + object[i].width, object[i].y + object[i].height);
		
		// create a rectangle around detected objects
		rectangle(roiObject, P1, P2, Scalar(0, 0, 255), 2);
		putText(roiObject, "Object", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
		// these values are calculated  using linear equations, for more details see pdf
		distObject = (-0.777)*(P2.x-P1.x) + 123.44;
		
		// show distance on screen
		ss.str(" ");
		ss.clear();
		ss<<"D = "<<distObject<<"cm";
		putText(roiObject, ss.str(), Point2f(1,130), 0,1, Scalar(0,0,255), 2);
    }
    
}

void TrafficDetection()
{
    roiTraffic = frameTraffic(Rect(200,0,200,140));
    cvtColor(roiTraffic, grayTraffic, COLOR_RGB2GRAY);
    equalizeHist(grayTraffic, grayTraffic);
    TrafficCascade.detectMultiScale(grayTraffic, traffic);
    
	// loop through all detected objects 
    for(int i=0; i<traffic.size(); i++)
    {
		Point P1(traffic[i].x, traffic[i].y);
		Point P2(traffic[i].x + traffic[i].width, traffic[i].y + traffic[i].height);
	
		rectangle(roiTraffic, P1, P2, Scalar(0, 0, 255), 2);
		putText(roiTraffic, "Traffic Light", P1, FONT_HERSHEY_PLAIN, 1,  Scalar(0, 0, 255, 255), 2);
		// these m and c values are calculated  using linear equations, for more details see pdf
		distTraffic = (-1.75)*(P2.x-P1.x) + 99.5;
		
		// show distance of traffic light from camera
		ss.str(" ");
		ss.clear();
		ss<<"D = "<<distTraffic<<"cm";
		putText(roiTraffic, ss.str(), Point2f(1,130), 0,1, Scalar(0,0,255), 2);
    }    
}

void wait(int seconds)
{
	auto tstart = std::chrono::system_clock::now();
	auto tend = std::chrono::system_clock::now();
	
	std::chrono::duration<double> telapsed_seconds = tend-tstart;
	while(telapsed_seconds.count() < seconds)
	{
	    
	    tend = std::chrono::system_clock::now();
	    telapsed_seconds = tend-tstart;
	}
}

int LoadMLModels()
{
	if(!StopCascade.load("//home//pi//Desktop//Machine Learning//Stop_cascade.xml"))
    {
		printf("Unable to open stop cascade file");
		return 0;
    }
    if(!ObjectCascade.load("//home//pi//Desktop//Machine Learning//Obstacle_cascade.xml"))
    {
		printf("Unable to open Object cascade file");
		return 0;
    }
    if(!TrafficCascade.load("//home/pi/Desktop/Machine Learning/Traffic_cascade.xml"))
    {
		printf("Unable to open traffic cascade file");
		return 0;
    }
	return 1;
}

void DefinePinConfig()
{
	// Using wiring pi pin 21,22,23,24 for communicating with Arduino
	pinMode(21, OUTPUT); 
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(24, OUTPUT);
}

int main(int argc,char **argv)
{
	/*
	This initialises wiringPi and assumes that the calling program is going to be using the wiringPi pin numbering scheme. This is a simplified numbering scheme which provides a mapping from virtual pin numbers 0 through 16 to the real underlying Broadcom GPIO pin numbers.
	For more info : https://projects.drogon.net/raspberry-pi/wiringpi/pins/
	*/
    wiringPiSetup();
	DefinePinConfig();
    
    if(!LoadMLModels())
		return 0;
	
	// Camera Setup
	Setup(argc, argv, camera);
	cout<<"Connecting to Camera"<<endl;
	if (!camera.open())
	{
		cout<<"Failed to Connect"<<endl;
		return 0;
    }
	cout<<"Camera Id = "<<camera.getId()<<endl;
	
	
    while(1)
    {
		// Used for FPS Calculation
		//auto start = std::chrono::system_clock::now();

		Capture();
		// Image processing
		Perspective();
		Threshold();
		// Calculation to find by how much we need to move our car to left/right or forward
		Histrogram();
		LaneFinder();
		LaneCenter();
		
		StopDetection();
		ObjectDetection();
		TrafficDetection();
		
		/******************************* MACHINE LEARNING DECISIONS ******************************************/
		// if distance of stop signal is less than 60 but greater than 25, stop the car by sending decimal 8 to arduino
		if (distStop > 25 && distStop < 60)
		{
			digitalWrite(21, 0);
			digitalWrite(22, 0);    //decimal = 8
			digitalWrite(23, 0);
			digitalWrite(24, 1);
			cout<<"Stop Sign"<<endl;
			distStop = 0;
			// once stop sign is detected do not do anything else
			goto STOP_SIGN;
		}
		// if distance of object is less than 45 but greater than 25, send the lane change signal to arduino, decimal 9
		if (distObject > 25 && distObject < 45)
		{
			digitalWrite(21, 1);
			digitalWrite(22, 0);    //decimal = 9
			digitalWrite(23, 0);
			digitalWrite(24, 1);
			cout<<"Object"<<endl;
			distObject = 0;
			// once the object is identified, don't do anything else as the lane change routine is being executed
			goto OBJECT;
		}
		// if distance of traffic is less than 50 but greater than 5, send the stop  signal to arduino, decimal 10
		if (distTraffic > 5 && distTraffic < 50)
		{
			digitalWrite(21, 0);
			digitalWrite(22, 1);    //decimal = 10
			digitalWrite(23, 0);
			digitalWrite(24, 1);
			cout<<"Traffic"<<endl;
			distTraffic = 0;
			// once the traffic is identified, don't do anything else until and unless the traffic is no more identified when user changes signal
			// to green
			goto TRAFFIC;
		}
		
		/******************************* LANE END DECISION ******************************************/
		// this 4000 value depends on the lighting condition and the size of horizontal strip you have 
		// Lane end was identified, send UTurn signal to arduino
		if (laneEnd > 4000)
		{
			digitalWrite(21, 1);
			digitalWrite(22, 1);    //decimal = 7
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Lane End"<<endl;
		}
		
		
		/******************************* IMAGE PROCESSING LANE DETECTION DECISIONS ******************************************/
		// Based on the value of result, we take decision such as move forward, move left, right.
		// if say, result is 0 we send signal by making a pin combination of 0 and sending the same to our pins.
		// Left right with a numeric suffix just means the amount of torque value we are sending to the motors from arduino
		if (result == 0)
		{
			digitalWrite(21, 0);
			digitalWrite(22, 0);    //decimal = 0
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Forward"<<endl;
		} 
		else if (result > 0 && result < 10)
		{
			digitalWrite(21, 1);
			digitalWrite(22, 0);    //decimal = 1
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Right1"<<endl;
		}
		else if (result >= 10 && result < 20)
		{
			digitalWrite(21, 0);
			digitalWrite(22, 1);    //decimal = 2
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Right2"<<endl;
		}
		else if (result >20)
		{
			digitalWrite(21, 1);
			digitalWrite(22, 1);    //decimal = 3
			digitalWrite(23, 0);
			digitalWrite(24, 0);
			cout<<"Right3"<<endl;
		}
		else if (result < 0 && result > -10)
		{
			digitalWrite(21, 0);
			digitalWrite(22, 0);    //decimal = 4
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Left1"<<endl;
		}
		else if (result <= -10 && result > -20)
		{
			digitalWrite(21, 1);
			digitalWrite(22, 0);    //decimal = 5
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Left2"<<endl;
		}
		else if (result <-20)
		{
			digitalWrite(21, 0);
			digitalWrite(22, 1);    //decimal = 6
			digitalWrite(23, 1);
			digitalWrite(24, 0);
			cout<<"Left3"<<endl;
		}
		
		STOP_SIGN:
		TRAFFIC:
		OBJECT:
		
		/******************************* DISPLAY THE DECISIONS ******************************************/
		// Show the decision "Move Forward", "Move Left", "Move Right", "U Turn" on the screen
		if (laneEnd > 4000) // this value was greater than 4000 was found to be intensity value of white horizontal strip in my case 
		{
		   ss.str(" ");
		   ss.clear();
		   ss<<" Lane End";
		   putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(255,0,0), 2);
		   cout<<"Lane End";
		}
		else if (result == 0)
		{
		   ss.str(" ");
		   ss.clear();
		   ss<<"Result = "<<result<<" Move Forward";
		   putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
		}
		else if (result > 0)
		{
		   ss.str(" ");
		   ss.clear();
		   ss<<"Result = "<<result<<" Move Right";
		   putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
		}
		else if (result < 0)
		{
		   ss.str(" ");
		   ss.clear();
		   ss<<"Result = "<<result<<" Move Left";
		   putText(frame, ss.str(), Point2f(1,50), 0,1, Scalar(0,0,255), 2);
		}
		
		
		/******************************* DISPLAY THE FRAMES ******************************************/
		// Show the frames on the screen
		namedWindow("Original", WINDOW_KEEPRATIO);
		moveWindow("Original", 0, 100);
		resizeWindow("Original", 640, 480);
		imshow("Original", frame);
		
		namedWindow("Perspective", WINDOW_KEEPRATIO);
		moveWindow("Perspective", 640, 100);
		resizeWindow("Perspective", 640, 480);
		imshow("Perspective", framePers);
		
		namedWindow("Final", WINDOW_KEEPRATIO);
		moveWindow("Final", 1280, 100);
		resizeWindow("Final", 640, 480);
		imshow("Final", frameFinal);
		
		namedWindow("Stop Sign", WINDOW_KEEPRATIO);
		moveWindow("Stop Sign", 1280, 580);
		resizeWindow("Stop Sign", 640, 480);
		imshow("Stop Sign", roiStop);
		
		namedWindow("Object", WINDOW_KEEPRATIO);
		moveWindow("Object", 640, 580);
		resizeWindow("Object", 640, 480);
		imshow("Object", roiObject);
		
		namedWindow("Traffic", WINDOW_KEEPRATIO);
		moveWindow("Traffic", 0, 580);
		resizeWindow("Traffic", 640, 480);
		imshow("Traffic", roiTraffic);
		
		waitKey(1);
	
		// Used for FPS Calculation
		/*
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> elapsed_seconds = end-start;
		float t = elapsed_seconds.count();
		int FPS = 1/t;
		cout<<"FPS = "<<FPS<<endl;
		*/
    }

    
    return 0;
     
}
