/*************************************************************************************
************************* C++ code to capture images *********************************
*************************************************************************************/

#include <opencv2/opencv.hpp>
#include <raspicam_cv.h>
#include <iostream>

using namespace std;
using namespace cv;
using namespace raspicam;

// Store the captured frame
Mat frame;

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



int main(int argc, char** argv)
{
	// camera initialization and setup
    RaspiCam_Cv camera;
    Setup(argc, argv, camera);
	
    cout<<"Connecting to camera"<<endl;
    if (!camera.open())
    {
		cout<<"Failed to Connect"<<endl;
		return 0;
    }
     
	cout<<"Camera Id = "<<camera.getId()<<endl;
	
	for (int i=0; i<100; i++)
	{
		// fetch the frame
		camera.grab();
		camera.retrieve(frame);
		// convert to grayscale
		cvtColor(frame,frame, COLOR_BGR2GRAY);
		// show image on screen
		imshow("Sample", frame);
		// save the image
		imwrite("Image_name_"+to_string(i)+".jpg" , frame);
		waitKey();
	}
     
    return 0;
}

