#include "transformView.hpp"

TransformView::TransformView() {
	vanishing_point = 153;
	translation_scale_factor = 0.0056;
	pre_processed = true;
}

bool TransformView::Load_Homography_LUT(string file_name, double *H_LUT, double * m, double * b){
	std::ifstream file_H(file_name.c_str());
	int itr = 0;
	if (file_H.is_open()){

		while (!file_H.eof()){
			file_H >> H_LUT[itr++];
		}
	} else {
		std::cout << "could not find "<< file_name<<" for rotation LUT" << std::endl;
		return false;
	}

	file_H.close();
	//std::cout << "H start " << H_LUT[0] << " last " << H_LUT[itr - 2] << std::endl;

	for (int i = 0; i < 9; i++) {
		double x[17] = {};
		double y[17] = {};
		double r = 0;
		x[0] = 0;
		y[0] = 0;
		for (int rot = 1; rot <= 16; rot++) {
			x[rot] = rot;
			y[rot] = H_LUT[((rot - 1) * 9) + i];
		}
		
		linreg(17, x, y, &m[i], &b[i], &r);
	}
	
	return true;
}
bool TransformView::set_Homography_rotation(cv::Mat & Hmatrix, double rotation_angle){
	if (rotation_angle > 16 || rotation_angle< -16){
		/*std::cout << "angle out of range" << std::endl;
		for (int i = 0; i < 9; i++){
			Hmatrix.at<double>(i) = Hplus[((16 - 1) * 9) + i];
		}
		Hmatrix = Hmatrix.t();*/
		return false;
	}
	
	else if (rotation_angle >0){ // positive rotation (clockwise)
		
		/*for (int i = 0; i < 9; i++){
			Hmatrix.at<double>(i) = Hplus[((rotation_angle - 1) * 9)+i];
		}*/
		for (int i = 0; i < 9; i++){
			Hmatrix.at<double>(i) = calculate_y(m_plus[i], b_plus[i], rotation_angle);
		}
		
		Hmatrix = Hmatrix.t();
	}
	else if (rotation_angle <0){ // negative rotation (counter clockwise)
		/*for (int i = 0; i < 9; i++){
			Hmatrix.at<double>(i) = Hneg[(((-1*rotation_angle) - 1) * 9) + i];
		}*/
		for (int i = 0; i < 9; i++){
			Hmatrix.at<double>(i) = calculate_y(m_neg[i], b_neg[i], fabs(rotation_angle));
		}
		
		Hmatrix = Hmatrix.t();
	}
	return true;
}
int TransformView::viewpoint_transformation(cv::Mat input_image, double rotation_angle, double Y_shift, cv::Mat &result){
	printf(" [ viewpoint_transformation ] - rotation angle :: %lf || Y_shift :: %lf\n", rotation_angle, Y_shift);
	//cv::Mat image1;
	cv::Mat ori_channel[3];
	cv::Mat res_channel[3];
	//cvtColor(input_image, image1, CV_RGB2GRAY);
	cv::split(input_image, ori_channel);
	//Mat image1 = imread("ab/0_0_1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	int m1 = ori_channel[0].rows;//image1.rows;
	int m2 = ori_channel[0].cols;//image1.cols;
	result = cv::Mat::zeros(m1, m2, CV_8U);
	for (int x = 0; x < 3; x++){
		res_channel[x] =  cv::Mat::zeros(m1, m2, CV_8U);
	}
	res_channel[0] = cv::Scalar(0);
	res_channel[1] = cv::Scalar(0);
	res_channel[2] = cv::Scalar(0);
	//cv::Mat mask = cv::Mat::zeros(m1, m2, CV_8U);
	Y_shift *= 100; // convert from meter to cm

	//1. Loading data
	cv::Mat Hrot = cv::Mat::eye(3, 3, CV_64F);
	//Hrot.at<double>(2, 2) = 10;
	//cout << "Hrot" << Hrot.at<double>(8) << endl;
	if (pre_processed){
		if (!Load_Homography_LUT("../vctxt/Hplus.txt", Hplus, m_plus, b_plus)) return 0;
		if (!Load_Homography_LUT("../vctxt/Hneg.txt", Hneg, m_neg, b_neg)) return 0;
		pre_processed = false;
	}
	

	//2. setup homography for rotation
	if (!set_Homography_rotation(Hrot, rotation_angle)) return 0;
	//std::cout << "Hrot " << Hrot << std::endl;
	/*cv::imshow("tes0", ori_channel[2]);
	cv::waitKey(10);*/

	//3.Warping and add shift
	for (int i = 0; i < m1; i++){
		for (int j = 0; j < m2; j++){
			cv::Mat hc = cv::Mat::ones(3, 1, CV_64F);
			hc.at<double>(0) = j;
			hc.at<double>(1) = i;
			cv::Mat temp = Hrot*hc;
			temp.at<double>(0) += Y_shift *translation_scale_factor * (temp.at<double>(1) - vanishing_point);
			int temp1 = ceil(temp.at<double>(0) / temp.at<double>(2));
			int temp2 = ceil(temp.at<double>(1) / temp.at<double>(2));
			if (temp1 < m2 && temp2 <m1 && temp1 >0 && temp2 >0){
				//result.at<unsigned char>(i, j) = image1.at<unsigned char>(temp2, temp1);
				for (int x = 0; x < 3; x++){
					res_channel[x].at<unsigned char>(i, j) = ori_channel[x].at<unsigned char>(temp2, temp1);
				}
			}

		}
	}
	
	
	//4. Copy image above horizon
	for (int i = 0; i < vanishing_point; i++){
		for (int j = 0; j < m2; j++){
			//result.at<unsigned char>(i, j) = image1.at<unsigned char>(i, j);
			for (int x = 0; x < 3; x++){
				res_channel[x].at<unsigned char>(i, j) = ori_channel[x].at<unsigned char>(i, j);
			}
		}
	}
	
	cv::merge(res_channel, 3, result);
	
	return 1;
}

int TransformView::linreg(int n, const double x[], const double y[], double* m, double* b, double* r)
{
	double   sumx = 0.0;                        /* sum of x                      */
	double   sumx2 = 0.0;                       /* sum of x**2                   */
	double   sumxy = 0.0;                       /* sum of x * y                  */
	double   sumy = 0.0;                        /* sum of y                      */
	double   sumy2 = 0.0;                       /* sum of y**2                   */

	for (int i = 0; i<n; i++)
	{
		sumx += x[i];
		sumx2 += sqr(x[i]);
		sumxy += x[i] * y[i];
		sumy += y[i];
		sumy2 += sqr(y[i]);
	}

	double denom = (n * sumx2 - sqr(sumx));
	if (denom == 0) {
		// singular matrix. can't solve the problem.
		*m = 0;
		*b = 0;
		if (r) *r = 0;
		return 1;
	}

	*m = (n * sumxy - sumx * sumy) / denom;
	*b = (sumy * sumx2 - sumx * sumxy) / denom;
	if (r != NULL) {
		*r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
			sqrt((sumx2 - sqr(sumx) / n) *
			(sumy2 - sqr(sumy) / n));
	}

	return 0;
}

double TransformView::calculate_y(double m, double b, double x){

	return (m*x + b);
}