#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(): p_error_(0.0), i_error_(0.0), d_error_(0.0), cur_pos_(0), cur_step_(0), is_initialized_(false)
{}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double cte) {
    koeffs_[0] = Kp;
    koeffs_[1] = Kd;
    koeffs_[2] = Ki;

    deltas_[0] = 0.01;
    deltas_[1] = 0.05;
    deltas_[2] = 0.0001;

    p_error_ = cte;
    i_error_ = cte;

    is_initialized_ = true;
    best_error_ = cte;
    iter_counter = 0;
    avg_error_ = 0.0;
}

void PID::UpdateError(double cte) {
    i_error_ += cte;
    d_error_ = cte - p_error_;
    p_error_ = cte;
}

double PID::TotalError() {
    double error = -koeffs_[0] * p_error_ - koeffs_[1] * d_error_ - koeffs_[2] * i_error_;
    return error;
}

void PID::ParamUpdate(int option) {
    switch (option) {
        case INCREASE_COEFF_ :
            koeffs_[cur_pos_] += deltas_[cur_pos_];
            cur_step_ += 1;
            break;
        case DOUBLE_DECREASE_COEFF_ :
            koeffs_[cur_pos_] += -2 * deltas_[cur_pos_];
            cur_step_ += 1;
            break;
        case INCREASE_COEFF_REDUCE_DELTA_ :
            koeffs_[cur_pos_] += deltas_[cur_pos_];
            DeltaUpdate(DECREASE_DELTA_);
            break;
        default:
            koeffs_[cur_pos_] += deltas_[cur_pos_];
            break;
    }
}

void PID::DeltaUpdate(double mult) {
    deltas_[cur_pos_] *= mult;
    // when we finished updating current coefficient, we update delta.
    // so here we switch to next coefficient and set update step to 0
    cur_pos_ = (cur_pos_ + 1) % 3;
    cur_step_ = 0;
    // OK, here is a good place to start updating the next coefficient too.
    ParamUpdate(INCREASE_COEFF_);
}

double PID::SumDeltas() {
    double result = 0.0;
    for (double delta : deltas_) {
        result += delta;
    }
    return result;
}

