#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "stdio.h"
#include <iostream>
// #include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>  // NOLINT(readability/streams)
#include <string>
#include <utility>
#include <vector>

#include "transformView.hpp"
#include "imageProcess.hpp"
#include "drivingCommon.hpp"

#define ORG_IMG_WIDTH 1280
#define ORG_IMG_HEIGHT 720
#define ROI_WIDTH 720
#define ROI_HEIGHT 300
#define ROI_X (ORG_IMG_WIDTH - ROI_WIDTH)/2
#define ROI_Y 153

using namespace cv;
using namespace std;

int main() {
	// Create imageProcess object
	imageProcess custom_imgproc; 
	// Create Transform view point object
    TransformView transformview;

	// Load Video
	string vc_path = "../vctxt/Center_KIAPI_2lane-1.avi";
    VideoCapture vc = VideoCapture(vc_path);
    
    if (!vc.isOpened()){
        printf(" ===== Video File Load Fail ===== \n");
        exit(-1);
    }

    // Load Vehicle Info Data
    string vc_info_path = "../vctxt/Center_KIAPI_2lane-1.txt";

    // Extract Ground truth
	vector<string> lines;
    vector<pair<double, double> > ground_truth;

    lines = extractVehicleInfo(vc_info_path);
    ground_truth = extractVehicleGroundTruth(lines);

    // extract random rotation and y-shift value
    srand((unsigned)time(NULL)); // initialize random seed
    const double distance_Center_to_Right = 0.535;
    const double distance_Center_to_Left = -0.535;
    double rotation = 0.0, y_shift = 0.0;
    double delta = 0.0;

    // roatation and shift label file save
    string save_file_name = "../practice/video1/rotation_and_shift.txt";
    ofstream fout;
    fout.open(save_file_name.c_str());

    string save_image_path = "";
	char temp[200];

    // define Mat variable
	Mat org_img;
	Mat input_img;
	Mat separate_img[3];
	Mat transform_view_img;
	Mat roi_img;

    Rect ROI = Rect(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT);

	int frame = 0;
	int key = 0;

	while(true) {
		printf("\n\n [ FRAME ] :: %d\n", frame);

		vc >> org_img;

		if(org_img.empty()) {
		    printf(" [ THIS VIDEO IS TERMINATED ] \n\n");
		    break;
		}

		custom_imgproc.separateImageTrainData(org_img, separate_img, ORG_IMG_WIDTH, ORG_IMG_HEIGHT);
		separate_img[1].copyTo(input_img);

		for (int i = 0; i < 10; i++) {

			save_image_path.clear();
			save_image_path.append("../practice/video1/Center_KIAPI_2lane-1");
			save_image_path.append(format("_%d_%d.png",frame,i));

			if (i == 0) {
				rotation = ground_truth[frame].first;
				y_shift = ground_truth[frame].second;
			} else {
				rotation = createRandomValue(16.0);
				y_shift = createRandomValue(1.0);
			}

			if (y_shift > distance_Center_to_Right / 2.){
			    cout << " - apply View Point :: RIGHT" << endl;
			    delta = y_shift - distance_Center_to_Right;
			    separate_img[2].copyTo(input_img); // right camera
			} else if (y_shift < distance_Center_to_Left / 2.){
			    cout << " - apply View Point :: LEFT" << endl;
			    delta = y_shift - distance_Center_to_Left;
			    separate_img[0].copyTo(input_img);  // left camera
			} else{
			    cout << " - apply View Point :: CENTER" << endl;
			    delta = y_shift;
			    separate_img[1].copyTo(input_img); // center camera
			}

			// transform view point
			transformview.viewpoint_transformation(input_img, rotation, delta, transform_view_img);
			roi_img = transform_view_img(ROI);

			imwrite(save_image_path, roi_img);
			imshow("input_img", roi_img);

			key = waitKey(1);

			// imagename, frame, rotation, y-shift
			sprintf(temp, "/practice/video1/Center_KIAPI_2lane-1_%d_%d.png",frame,i);
			save_image_path = temp;

			fout << save_image_path << "," << frame << "," << rotation << "," << y_shift << endl;
		}



		if (key==27 || key == 1048603) { //ESC for Linux key :: 1048603
		    break;
		}

		frame++;
	}

	fout.close();

	return 0;
}
