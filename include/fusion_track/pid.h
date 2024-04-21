// PidController.h
#include <morai_msgs/CtrlCmd.h>


#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PidController {
public:
    PidController(double p_gain, double i_gain, double d_gain, double control_time);
    double pid(double target_vel, double current_vel);

private:
    double p_gain_;
    double i_gain_;
    double d_gain_;
    double control_time_;
    double prev_error_;
    double i_control_;
    double d_control_;
    double prev_lpf_;
    double x_;
};

#endif  // PID_CONTROLLER_H