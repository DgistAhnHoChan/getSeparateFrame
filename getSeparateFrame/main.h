#include <fstream>
#include <stdio.h>

#include <opencv2\opencv.hpp>
#include <opencv2\highgui\highgui.hpp>

#define PATH_VIDEO "../vctxt/camera.avi"
#define PATH_DATA "../vctxt/text.txt"

using namespace cv;
using namespace std;


vector<string> extractVehicleInfo(string file_name);



ifstream input;
ofstream output;

VideoCapture vc;
