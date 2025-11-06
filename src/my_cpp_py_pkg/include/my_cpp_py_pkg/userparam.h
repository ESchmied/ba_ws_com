#ifndef USERPARAM_H
#define USERPARAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct{

    // MPC parameters
    int NX;
    int NU;
    int NH;

    double dt;
    int Nhor;
    double Thor;

    // Integral cost weights
    double Q_pos;
    double Q_theta;
    double Q_vel;

    // Terminal cost weights
    double Q_pos_T;
    double Q_theta_T;
    double Q_vel_T;

    // Control effort weights
    //warum R?
    double R_steer;
    double R_accel;

    // Model parameters
    double max_velocity;
    double v_switch;

    double yaw_min;
    double yaw_max;
    double acc_min;
    double acc_max;

    double wheelbase;
    double m;
    double lf;
    double lr;
    double C_af;
    double C_ar;
    double I_z;

    // Others
    double centerline_dist;
    bool soft_constraints;
    double* ref_traj;
    int ref_length;

    double* inner_border;
    int inner_border_len;
    double* outer_border;
    int outer_border_len;

} UserParam;

#ifdef __cplusplus
}
#endif

#endif //USERPARAM_H