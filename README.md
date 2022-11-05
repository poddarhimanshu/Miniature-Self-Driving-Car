## About

The Car model has Raspberry PI as the master device, which receives real time data from the camera mounted on the top of the car. Decisions are taken on the basis of each frame that is received as input, the frame is pre-processed and the decisions accordingly are sent to the Slave device Arduino. The command to the Arduino basically containing inputs like "Move Forward", "Move Left", "Move Right", "Stop", "Take U Turn", "Obstacle ahead", "Lane Change", "Stop", "Wait".

The car keeps moving and circles the track until the program is made to stop to run on the RPI. In its run, the car in a loop identifies the lane, if an obstacle is encountered, it changes its lane to go the other lane and then gets back on its own track. When Lane End is identified, the Car itself takes a UTurn to go the other Lane. If a traffic signal with a RED light is identified the car stops until and unless the light turns green. When a stop signal is detected by our camera our car waits for few seconds and then starts moving forward. Moves forward and when the lane is over it takes a UTurn and goes to the other side of the road.

The machine learning model for obstacle was currently trained using 100 positive images of obstacle and 600 negative images of the surrounding.

The machine learning model for traffic signal was trained using 100positive images of traffic signal with red light, and traffic signal with no light on. I had no light on as positive images because we want the car to stop if a traffic signal is detected but does not show us any signal. I used 700 negative images in this case containing images of our surrounding and traffic signal with green light on.

To detect the stop signal I used 100 positive images of stop signal and 300 negative images of the surrounding.

If someone needs the images to get the idea of training, please let me know.

One question which I often get asked is, say my camera is at some far distance and before reaching some point, what if your model identifies it from a far distance and stop?

Well the answer is, yes it will happen and the car does stop, but I have taken care of that by finding the distance of the signal from my car using the linear equations, as you are pretty much aware the detection of objects would create a rectangle/square around my object, the size of the square will vary based on the distance of the object from the camera, the distance-object size follows the linear equation in 2 variables using which we can get the value of m and c, and based on the input pixel size of our detection we can get the distance of object from my car. Then by thresholding the same to say some distance X, I am stopping my car currently.

## Working and Behind the Scenes

Please find the entire playlist of [Lane detection], [Lane and Object detection], [Lane and Stop Signal detection], [Lane, Object and Stop signal detection], [Lane, Object, Stop and Traffic signal detection] here.

https://www.youtube.com/watch?v=6ToTwucgpss&list=PLHEVRs_P8mBQedxKYZu_b6-k93mOZXC79

#### To know more about the project implementation. Please go through the documentation file "SelfDrivingCarDocumentation.pdf"

## Code Files
<ul>
<li>The code running on Raspberry PI can be found here : Miniature Self Driving Car\RaspberryPI\RaspberryPI.cpp</li><br>
<li>The code running on Arduino can be found here : Miniature Self Driving Car\Arduino\CarMovement\CarMovement.ino</li><br>
<li>The code to capture positive and negative images of objects can be found here : Miniature Self Driving Car\CaptureImages\CaptureImages.cpp</li>br>
