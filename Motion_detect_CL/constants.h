#pragma once
#include "main.h"

#define ESCAPE_KEY 27
//#define ENTER_KEY 13
//grayscale threshold for BW conversion
//#define MASK_THRESH 20


//filter options
//dilation options
#define DILATE_SIZE 5 

//erode sensitivity
#define ERODE_SIZE 3

//max buffer size of image (w x h x bytes per pixel)
#define IMAGE_BUFFER_SIZE (FRAME_HEIGHT * FRAME_WIDTH * 4)

//window titles
#define original_image "Original Image"
#define filter_mask "Filter"


//max resolution
const int FRAME_HEIGHT = 480;
const int FRAME_WIDTH = 640;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 500;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH / 2;


//MOG2 constants
const float varThresholdGen = 11.0f;
const float backgroundRatio = 0.7f;

//#define MOG_HISTORY 500

//const float varThreshold = 16.0f;

//crosshair parameters
#define CROSSHAIR_RADIUS 30
#define COLOR_GREEN Scalar(0, 255, 0)


//noise removal of the mask
#define NOISE_H 10
#define NOISE_TEMPLATE_WINDOW 7
#define NOISE_SEARCH_WINDOW 21

//color red
#define COLOR_RED Scalar(0, 0, 0xff)

#define COLOR_BLUE Scalar(0xff, 0, 0)



#define MODULO_256 ((1<<8)-1)