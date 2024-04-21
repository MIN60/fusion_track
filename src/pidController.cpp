#include "fusion_track/pid.h"
#include <morai_msgs/CtrlCmd.h>

// PID 구현
// P(Proportional): 오차 비례 제어
// I(Integral): 오차 누적값 비례 제어
// D(Derivative): 오차 변화율 비례 제어

// target_vel과 current_vel 사이의 오차를 계산
// 오차를 바탕으로 PID 제어를 수행하여 최종 출력값을 계산
// 계산된 출력값은 후에 Low-pass filter를 통과하여 반환
// 잉 복잡해

PidController::PidController(double p_gain, double i_gain, double d_gain, double control_time)
    : p_gain_(p_gain), i_gain_(i_gain), d_gain_(d_gain), control_time_(control_time),
      prev_error_(0), i_control_(0), d_control_(0), prev_lpf_(0), x_(0) {}


double PidController::pid(double target_vel, double current_vel) {
    double error = target_vel - current_vel; //오차 계산

    // P, I, D 제어
    double p_control = p_gain_ * error;
    i_control_ += i_gain_ * error * control_time_;
    d_control_ = d_gain_ * (error - prev_error_) / control_time_;

    double output = p_control + i_control_ + d_control_;

    // Low-pass filter
    double alpha = 0.3;
    x_ = output;
    double lpf = alpha * prev_lpf_ + (1 - alpha) * x_;

    // Update previous values
    prev_error_ = error;
    prev_lpf_ = lpf;

    return lpf;
}

/*
self.p_gain=1 
self.i_gain=4 #0.5   #steady state error
self.d_gain=0.35 #0.8 #overshoot

self.controlTime=0.1
self.prev_error=0
self.i_control=0
*/