#ifndef PID_H
#define PID_H

class PID {
private:
    double kp;
    double ki;
    double kd;
    double ep;
    double ei;
    double ed;
    double dt;

public:
    PID(double kp, double ki, double kd);

    //update the bias
    void update_e(double e);

    //output the control u
    double get_u();

    double get_t();
};

#endif
