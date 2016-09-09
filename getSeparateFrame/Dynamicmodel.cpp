#include "Dynamicmodel.hpp"

const double recovery_time = 2.0;

Dynamicmodel::Dynamicmodel() {
    recovery_threshold_ypos = 0.1;
    recovery_threshold_angle = 0.5;//0.01;
    smooth_factor = 0.05;
    smooth_factor_for_output = 1;
}


double Dynamicmodel::Beta(double df, int angle_type){
    double angle_converter = 1;
    if (angle_type == 2){
        angle_converter = 180 / PI;
    }
    double tandf = tan(df / angle_converter);
#if CALCULATE_SIDESLIP
    return (atan(tandf*LF_per_Ltotal)*angle_converter);
#else
    return 0.;
#endif // CALCULATE_SIDESLIP
}

void Dynamicmodel::Rotation_and_Shift(double t0, double tn, double v0, double vn, double df, double currentYaw, double &Out_Yaw, double &Out_Xshift, double &Out_Yshift, int angle_type){
    vn /= 3.6;
    v0 /= 3.6;
    double numerator = (((tn - t0)*(vn - v0) / 2.) + ((tn - t0)*v0)) ;
    //printf("numerator : %lf\n", numerator);
    double angle_converter = 1;
    if (angle_type == 2){
        angle_converter = 180 / PI;
    }
    //printf("Beta : %lf \n", Beta(df, angle_type));
    Out_Yaw =    numerator * tan(df / angle_converter) * cos(Beta(df, angle_type) / angle_converter) *angle_converter / WHEELBASE_LENGTH;
    Out_Xshift = numerator * cos((currentYaw + Out_Yaw/2 + Beta(df, angle_type)) / angle_converter);
    Out_Yshift = numerator * sin((currentYaw + Out_Yaw/2 + Beta(df, angle_type)) / angle_converter);
}

void Dynamicmodel::adjustment_steering_angle(double rec_steering_command, double dt, double v0, double vn, double angle, double Y_shift, double desired_angle, double desired_Yposition, double &Out_DS, int angle_type){
    double delta_Y = desired_Yposition - Y_shift;
    double delta_Yaw = desired_angle - angle;
    
    if (fabs(delta_Y) < recovery_threshold_ypos && fabs(delta_Yaw)< recovery_threshold_angle) {
        Out_DS = rec_steering_command;
    } else {
        double smallest_dY = 9999999999.;
        double smallest_dYaw = 9999999999.;
        double min_sa =MIN_STEERING_DEGREE,  max_sa= MAX_STEERING_DEGREE;
        
        double Out_DS1 = 0, Out_DS2 = 0;

        for (double sa = min_sa; sa <= max_sa; sa += 0.1){
                
            if (angle_type == 1){
                sa *= PI / 180;
            }
            double df = sa / STEERING_RATIO;
            double tempYaw = 0, tempX = 0, tempY = 0;
            Rotation_and_Shift(0, dt, v0, vn, df, angle, tempYaw, tempX, tempY, angle_type);
            double dY = fabs(delta_Y*smooth_factor- tempY);
            double dYaw = fabs(delta_Yaw*smooth_factor- tempYaw);
            
            if (dY == 0 && dYaw == 0){
                Out_DS = sa;
                break;
            }
            if (dY < smallest_dY){
                smallest_dY = dY;
                Out_DS1 = sa;
            }
            if (dYaw < smallest_dYaw){
                smallest_dYaw = dYaw;
                Out_DS2 = sa;
            }           
        }
        Out_DS = (Out_DS1 + Out_DS2) * smooth_factor_for_output  / 2;
    }
}