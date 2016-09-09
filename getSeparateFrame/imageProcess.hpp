#include <stdio.h>

// OpenCV 헤더
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <string>
#include <utility>

// Namespace
using namespace cv;
using namespace std;

// Define(이미지)
#define NUM_CAMERA	4

class imageProcess {
public:
	void separateImage(Mat src, Mat *dst, int image_width, int image_height);
	void separateImageTrainData(Mat src, Mat *dst, int image_width, int image_height);
	void convertColor(Mat *src, Mat *dst);
};




