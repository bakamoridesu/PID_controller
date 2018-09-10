#ifndef PID_H
#define PID_H

class PID {
public:
    /*
    * Errors
    */
    double p_error_;
    double i_error_;
    double d_error_;

    //Coefficients
    double koeffs_[3];
    //Delta coeffitients
    double deltas_[3];
    // options for coefficient update
    static const int INCREASE_COEFF_ = 0;
    static const int DOUBLE_DECREASE_COEFF_ = 1;
    static const int INCREASE_COEFF_REDUCE_DELTA_ = 2;

    static constexpr double INCREASE_DELTA_ = 1.1;
    static constexpr double DECREASE_DELTA_ = 0.9;
    // current coeffitient being updated
    int cur_pos_;

    double best_error_;

    /*
     * current coefficient update step
     * may take values of 0,1,2,3
     * 0 : we didn't tweak the coefficient yet. Next : Try to increase by corresponding delta
     * 1 : we just tried to increase the coefficient. Next : Accept it or decrease.
     * 2 : we tried to increase and decrease the coefficient.
     * Next : Accept it or return previous coefficient value and reduce delta.
     */
    int cur_step_;

    bool is_initialized_;

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double cte);

    void ParamUpdate(int option);

    void DeltaUpdate(double mult);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
    double SumDeltas();

    unsigned long long iter_counter;
    double avg_error_;
};

#endif /* PID_H */
