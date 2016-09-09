#include <stdio.h>

// OpenCV 헤더
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <fstream>  // NOLINT(readability/streams)
#include <string>
#include <utility>
#include <vector>

#include "Dynamicmodel.hpp"

// Namespace
using namespace cv;
using namespace std;

vector<string> extractVehicleInfo(string file_name);
vector<pair<double, double> > extractVehicleGroundTruth(vector<string> lines);

double createRandomValue(double range);
double round(float value, int pos);

std::vector<std::string> &str_split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> str_split(const std::string &s, char delim);