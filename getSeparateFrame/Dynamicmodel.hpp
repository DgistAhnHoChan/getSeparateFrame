#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <fstream>  // NOLINT(readability/streams)
#include <string>
#include <utility>
#include <vector>
using namespace std;

#define CALCULATE_SIDESLIP 1
#define LF_per_Ltotal 0.6
#define WHEELBASE_LENGTH 2.64 // in meter
#define MAX_STEERING_DEGREE 540
#define MIN_STEERING_DEGREE -540
#define REAR_THREAD 1.615
#define STEERING_RATIO 12.857
#define PI 3.14159265359
#define RADIAN 1
#define DEGREE 2

class Dynamicmodel {

//public variable
public:
    double recovery_threshold_ypos;
    double recovery_threshold_angle;//0.01;
    double smooth_factor;
    double smooth_factor_for_output;

//public function
public:
    Dynamicmodel();
    double Beta(double df, int angle_type);

// y-axis is lateral movement (side)
// x-axis is longitudinal movement (forward)

/*
*
   Funtion : Rotation and shift 
   Input :
            t0 = time on initial position       -> Unit : sec
            tn = time on next position          -> Unit : sec
            v0 = velocity on initial position   -> Unit : kmph (km per hour)
            vn = velocity on next position      -> Unit : kmph (km per hour)
            df = front wheel angle = (steering angle / steering ratio)  -> Unit : radian or degree
            currentYaw = current heading angle of car  -> Unit : radian or degree
            angle_type = RADIAN or DEGREE (Default : RADIAN)

    Output :
            Out_Yaw = rotation angle (Yaw)              -> Unit : radian or degree
            Out_Xshift = shift on longitudinal axis (x) -> Unit : meter
            Out_Yshift = shift on lateral axis (y)      -> Unit : meter

    Example :
        Rotation_and_Shift(0, 5, 60, 70, 100/STEERING_RATIO, Yaw, Xshift, Yshift, DEGREE)
*/
void Rotation_and_Shift(double t0, double tn, double v0, double vn, double df, double currentYaw, double &Out_Yaw, double &Out_Xshift, double &Out_Yshift, int angle_type);

/*
*
    Funtion : adjustment_steering_angle
    Input :
            current_Yaw = current heading angle  (caused by random rotation) -> Unit : radian or degree
            current_Ypos = current lateral position (caused by random shift) -> Unit : meter 
            v0   = velocity on next position ->Unit : kmph (km per hour)
            vn   = volcity on desired position> Unit : kmph (km per hour)
            t0   = initial time (sec)
            tn   = time in desired postion 
            rec_steering_angle = current recorded steering angle -> Unit : degree or radian
            angle_type = RADIAN or DEGREE (Default : RADIAN)

    Output :
            desired steering angle for adjust to shift or rotation -> Unit : degree or radian

Example :
        adjustment_steering_angle(rec_steering_angle[i - 1], 1. / fps, rec_speed[i - 1], rec_speed[i], Yaw_ori, Y_pos, des_steering_angle, DEGREE);
*/

//void adjustment_steering_angle(double rec_steering_command, double dt, double v0, double vn, double angle, double Y_shift, double &Out_DS, int angle_type);
void adjustment_steering_angle( double rec_steering_command, double dt, double v0, double vn, double angle, double Y_shift, double desired_angle, double desired_Yposition, double &Out_DS, int angle_type);
};