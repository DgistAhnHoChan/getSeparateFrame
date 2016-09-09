#include "drivingCommon.hpp"

double round( float value, int pos ) {
    int p = pow(10, pos);
    float tmpValue;
    float result;
    if (value < 0) {
        tmpValue = abs(value);
    } else {
        tmpValue = value;
    }
    result = floor((tmpValue * p) + 0.5f) / p;

    if (value < 0) {
        result = result * -1;
    }

    return result;
}

// Extraction in a line
vector<string> extractVehicleInfo(string file_name) {
    vector<string> lines;
    string line;

    ifstream infile(file_name.c_str());

    while (std::getline(infile, line)) {
        lines.push_back(line);
    }

    return lines;
}

// extract ground truth
vector<pair<double, double> > extractVehicleGroundTruth(vector<string> lines) {

    Dynamicmodel dynamicmodel;

    vector<pair<double, double> > ground_truth;
    vector<string> pre_tokens;
    vector<string> cur_tokens;
    
    double v0 = 60.0; //kph
    double v1 = 60.0; //kph
    double t0 = 0.; // sec (default fix)
    double t1 = 0.1;// sec (default fix)
    int steer_angle;
    double yaw = 0., x_shift = 0., y_shift = 0.;

    int frame = 0;
    while(1) {
        if (frame >= 1) {
            pre_tokens.clear();
            cur_tokens.clear();

            pre_tokens = str_split(lines[frame-1], ',');
            cur_tokens = str_split(lines[frame], ',');

            v0 = (atof(pre_tokens[3].c_str()) + atof(pre_tokens[4].c_str()))/2.0;
            v1 = (atof(cur_tokens[3].c_str()) + atof(cur_tokens[4].c_str()))/2.0;
            steer_angle = atof(cur_tokens[1].c_str());

            dynamicmodel.Rotation_and_Shift(t0, t1, v0, v1, steer_angle / STEERING_RATIO, ground_truth[frame-1].first, yaw, x_shift, y_shift, DEGREE);
        }
        ground_truth.push_back(std::make_pair(yaw, y_shift));

        frame++;

        if(frame >= lines.size()) {
            break;
        }
    }

    return ground_truth;
}

double createRandomValue(double range) {
    double rand_value = 0.0;

    rand_value = (rand()/(float) RAND_MAX * (range * 2)) - range;

    return round(rand_value, 6);;
}

std::vector<std::string> &str_split(const std::string &s, char delim, std::vector<std::string> &elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> str_split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    str_split(s, delim, elems);
    return elems;
}

