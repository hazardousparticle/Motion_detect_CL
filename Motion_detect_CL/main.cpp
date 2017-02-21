#include <iostream>
#include <cstring>
#include <Windows.h>

#include "main.h"
#include "constants.h"

using namespace cv;
using namespace std;

bool trackFilteredObject(int &x, int &y, UMat threshold, UMat &cameraFeed);
void drawObject(int x, int y, Mat &frame);


int main(int argc, char* argv[])
{
	//set up the kinect for use as our video device
	INuiSensor * sensor;
	
	if (NuiCreateSensorByIndex(0, &sensor) < 0)
	{
		cout << "Kinect error." << endl;
		return 0;
	}

	HANDLE rgbStream, depthStream;
	HANDLE next_frame = CreateEventW(NULL, TRUE, FALSE, NULL);//signal when a frame is ready

	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	//color image
	sensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,            // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,    // Image resolution
		0,      // Image stream flags, e.g. near mode
		1,      // Number of frames to buffer
		next_frame,   // Event handle
		&rgbStream);

	//depth image
	sensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,            // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,    // Image resolution
		0,      // Image stream flags, e.g. near mode
		1,      // Number of frames to buffer
		NULL,   // Event handle
		&depthStream);

	sensor->NuiImageStreamSetImageFrameFlags(depthStream, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE | NUI_IMAGE_STREAM_FLAG_DISTINCT_OVERFLOW_DEPTH_VALUES);

	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;


	NUI_IMAGE_FRAME depthFrame;
	NUI_LOCKED_RECT LockedRectDepth;

	cout << "Kinect started." << endl;
	
	
	
	//start opencl
	if (!ocl::haveOpenCL())
	{
		cout << "No OpenCL" << endl;

		sensor->Release();
		return 0;
	}
	
	cv::ocl::Context context;
	if (!context.create(cv::ocl::Device::TYPE_GPU))
	{
		cout << "Failed creating the OCL context..." << endl;

		sensor->Release();

		return 0;
	}
	

	// In OpenCV 3.0.0 beta, only a single device is detected.
	cout << context.ndevices() << " GPU devices are detected." << endl;
	for (unsigned int i = 0; i < context.ndevices(); i++)
	{
		cv::ocl::Device device = context.device(i);
		cout << "name                 : " << device.name() << endl;
		cout << "available            : " << device.available() << endl;
		cout << "imageSupport         : " << device.imageSupport() << endl;
		cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
		cout << endl;
	}

	// Select the first device
	cv::ocl::Device(context.device(0));

	Mat frame = Mat(FRAME_HEIGHT,FRAME_WIDTH,CV_8UC4);//somewhere to store the camera images
	UMat mask = UMat(FRAME_HEIGHT, FRAME_WIDTH,USAGE_DEFAULT);//somewhere to store the mask we generate
	
	Ptr<BackgroundSubtractorMOG2> bgMog = createBackgroundSubtractorMOG2();

	bgMog[0].setVarThresholdGen(varThresholdGen);
	bgMog[0].setBackgroundRatio(backgroundRatio);

	cout << "Background subtractor ready" << endl;


	UMat erodeElement = getStructuringElement(MORPH_RECT, Size(ERODE_SIZE, ERODE_SIZE))
		.getUMat(cv::ACCESS_RW, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
	//dilate with larger element so make sure object is nicely visible
	UMat dilateElement = getStructuringElement(MORPH_RECT, Size(DILATE_SIZE, DILATE_SIZE))
		.getUMat(cv::ACCESS_RW, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

	char key = 0;//store the last key pressed

	namedWindow(original_image, CV_WINDOW_AUTOSIZE); //Create output window 

	//coords of tracked object
	int x = 0;
	int y = 0;
	int z = 0;

	const unsigned char * pFrame;//point to the image buffer used for copying
	const unsigned short * pDepth;

	ocl::setUseOpenCL(true);


	//UMat background = UMat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);

	while (key != ESCAPE_KEY)
	{
		//capture from kinect
		if (WaitForSingleObject(next_frame, 0) == WAIT_OBJECT_0)
		{
			if (sensor->NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame) != S_OK)
			{
				continue;
			}


			INuiFrameTexture* texture = imageFrame.pFrameTexture;
			texture->LockRect(0, &LockedRect, NULL, 0);

			if (!LockedRect.Pitch)
			{
				continue;
			}

			pFrame = (const unsigned char*)LockedRect.pBits;


			memcpy_s(frame.data, IMAGE_BUFFER_SIZE, pFrame, IMAGE_BUFFER_SIZE);

			//color equalisation
			//should adjust for brightness spikes
			/*cv::cvtColor(frame, frame, CV_BGR2YUV);
			vector<cv::Mat> channels;
			cv::split(frame, channels);
			equalizeHist(channels[0], channels[0]);
			cv::merge(channels, frame);
			cv::cvtColor(frame, frame, CV_YUV2BGR);*/

			key = cvWaitKey(1);
			//Sleep(1);

			//ready for next frame
			texture->UnlockRect(0);
			sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
			
			UMat frameToProcess = frame.getUMat(cv::ACCESS_RW, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
		
			//continue;
			if (!frameToProcess.empty())
			{
				try
				{

					//mog2_subtractor.operator ()(frame, mask);
					//bg.operator()(frame2, mask);
					bgMog[0].apply(frameToProcess, mask);

					//bgMog->getBackgroundImage(background);

					//remove noise form the mask
					fastNlMeansDenoising(mask, mask, NOISE_H, NOISE_TEMPLATE_WINDOW, NOISE_SEARCH_WINDOW);

					//imshow("noise gate", mask);

					erode(mask, mask, erodeElement);
					//erode(mask, mask, erodeElement);

					dilate(mask, mask, dilateElement);
					//dilate(mask, mask, dilateElement);

					//absdiff(frame,BackGnd,mask);
					//cvtColor(mask, mask, CV_BGR2GRAY);


					threshold(mask, mask, 0, 255, CV_THRESH_BINARY);

					//mask = mask_cl>0;//remove greys
					imshow(filter_mask, mask);

					if (trackFilteredObject(x, y, mask, frameToProcess))
					{
						//only get depth if there was an object
						
						sensor->NuiImageStreamGetNextFrame(depthStream, 0, &depthFrame);
						texture = depthFrame.pFrameTexture;
						texture->LockRect(0, &LockedRectDepth, NULL, 0);

						if (LockedRect.Pitch)
						{
							pDepth = (const unsigned short*)LockedRectDepth.pBits;//pointer to start of frame

							unsigned short depth = NuiDepthPixelToDepth(*pDepth + (x*FRAME_WIDTH + y));

							string t ="Tracked object Distance: " + std::to_string(depth ) + "mm";

							const unsigned short *endDepth = pDepth + (FRAME_HEIGHT * FRAME_WIDTH);

							//go through each pixel in the array of depths and add it to the grayscale
							Mat grey = Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC3);

							grey = Mat::zeros(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC3);

							typedef cv::Point3_<uint8_t> Pixel;
							for (Pixel &p : Mat_<Pixel>(grey))
							{
								if (pDepth > endDepth)
								{
									break;
								}
								depth = NuiDepthPixelToDepth(*pDepth++);

								//if (depth == TOO_FAR)
								//if depth == too close
								
								p.x = (unsigned char)(depth & MODULO_256);
								p.y = p.x;
								p.z = p.x;

							}
							
							imshow("depth", grey);


							Mat temp = Mat::zeros(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC4);
							cv::putText(temp, t, Point(50, 50), FONT_HERSHEY_PLAIN, 2, COLOR_BLUE, 2);

							//imshow("", temp);

							add(frameToProcess, temp, frameToProcess);

						}

						sensor->NuiImageStreamReleaseFrame(depthStream, &depthFrame);

						
					}


					//get the distance of the disturbance

					imshow(original_image, frameToProcess);

					//imshow("Background", background);
				}
				catch (const cv::Exception& e)
				{
					cout << e.what() << endl;
				}

			}


		}
	}

	cout << "shutting down sensor." << endl;

	cv::destroyAllWindows();

	sensor->Release();

	//CloseHandle(next_frame);

	return 0;
}


void drawObject(int x, int y, Mat &frame) {
	//draw a crosshair

	circle(frame, Point(x, y), CROSSHAIR_RADIUS - 5, COLOR_GREEN, 2);

	//horizontal line in cursor
	line(frame, Point(x - CROSSHAIR_RADIUS / 2, y), Point(x + CROSSHAIR_RADIUS / 2, y), COLOR_GREEN, 2);

	//vertical line in cursor
	line(frame, Point(x, y - CROSSHAIR_RADIUS / 2), Point(x, y + CROSSHAIR_RADIUS / 2), COLOR_GREEN, 2);
}


bool trackFilteredObject(int &x, int &y, const UMat threshold, UMat &cameraFeed)
{
	//Mat temp;
	//threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(threshold, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	//double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0)
	{
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS){

			double maxArea = 0;
			int largest_index = 0;

			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//find largest object
				if (area >= maxArea)
				{
					maxArea = area;
					largest_index = index;
				}
			}

			Moments moment = moments((cv::Mat)contours[largest_index]);
			//if the area is less than 20 px by 20px then it is probably just noise
			//if the area is the same as the 3/2 of the image size, probably just a bad filter
			//we only want the object with the largest area so we safe a reference area each
			//iteration and compare it to the area in the next iteration.
			if (maxArea>MIN_OBJECT_AREA && maxArea<MAX_OBJECT_AREA)
			{
				x = (int) moment.m10 / maxArea;
				y = (int) moment.m01 / maxArea;
				objectFound = true;
			}
			else
			{
				objectFound = false;
			}


			//let user know you found an object
			if (objectFound == true){
				//putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				
				Mat temp = Mat::zeros(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC4);
									
				drawObject(x, y, temp);

				add(cameraFeed, temp, cameraFeed);
			}
		}
	}


	return objectFound;
}
