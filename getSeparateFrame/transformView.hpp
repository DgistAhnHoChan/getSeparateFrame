#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

inline static double sqr(double x) {
	return x*x;
}
//int linreg(int n, double x[], double y[], double* b, double* m, double* r);
//double calculate_y(double m, double b, double x);

class TransformView {
public:
	double Hplus[150];
	double Hneg[150];
	int vanishing_point;
	double translation_scale_factor;
	bool pre_processed;
	double m_neg[9];
	double m_plus[9];
	double b_plus[9];
	double b_neg[9];

public:
//public function
/*
Funtion : viewpoint_transformation
Input :
	input_image = image to be transformed (Mat)
	rotation_angle = rotate image (range -16 ~ 16 degree)
	Y_shift = translation in lateral axis ( meter)

	Output : result = transformed image (Mat)
*/
	TransformView();
	int viewpoint_transformation(cv::Mat input_image, double rotation_angle, double Y_shift, cv::Mat &result);

private:
	bool Load_Homography_LUT(string file_name, double *H_LUT, double * m, double * b);
	bool set_Homography_rotation(cv::Mat & Hmatrix, double rotation_angle);
	int linreg(int n, const double x[], const double y[], double* m, double* b, double* r);
	double calculate_y(double m, double b, double x);
};


