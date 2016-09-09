#include "imageProcess.hpp"

void imageProcess::separateImage(Mat src, Mat *dst, int image_width, int image_height)
{
	vector<Mat> channels(3);
	split(src, channels);
	dst[0] = channels[0]( Rect(0,	0,	image_width, image_height) ).clone();
	dst[1] = channels[0]( Rect(image_width, 0,	image_width, image_height) ).clone();
	dst[2] = channels[0]( Rect(0,	image_height,	image_width, image_height) ).clone();
	dst[3] = channels[0]( Rect(image_width,	image_height,	image_width, image_height) ).clone();
}

void imageProcess::separateImageTrainData(Mat src, Mat *dst, int image_width, int image_height)
{
	dst[0] = src( Rect(0,	0,	image_width, image_height) ).clone();
	dst[1] = src( Rect(image_width, 0,	image_width, image_height) ).clone();
	dst[2] = src( Rect(image_width*2,	0,	image_width, image_height) ).clone();
}

void imageProcess::convertColor(Mat *src, Mat *dst)
{
	for(int idx=0; idx<NUM_CAMERA; idx++)
	cvtColor( src[idx], dst[idx], CV_BayerGB2BGR );	// Opencv의 컬러 공간 변환을 사용
}