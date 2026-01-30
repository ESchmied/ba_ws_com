/* This file is part of GRAMPC - (https://sourceforge.net/projects/grampc/)
 *
 * GRAMPC -- A software framework for embedded nonlinear model predictive
 * control using a gradient-based augmented Lagrangian approach
 *
 * Copyright 2014-2019 by Tobias Englert, Knut Graichen, Felix Mesmer,
 * Soenke Rhein, Andreas Voelz, Bartosz Kaepernick (<v2.0), Tilman Utz (<v2.0).
 * All rights reserved.
 *
 * GRAMPC is distributed under the BSD-3-Clause license, see LICENSE.txt
 *
 *
 *
 *
 *
 *
 *
 * This probfct file provides an interface to GRAMPC. The most general
 * formulation of the optimal control problem (OCP) that can be solved
 * by GRAMPC has the following structure
 *                                           _T
 *	                                        /
 *      min    J(u,p,T;x0) = V(T,x(T),p) + / l(t,x(t),u(t),p) dt
 *   u(.),p,T                            _/
 *                                      0
 *              .
 *      s.t.   Mx(t) = f(t0+t,x(t),u(t),p), x(0) = x0
 *             g(t,x(t),u(t),p)  = 0,   gT(T,x(T),p)  = 0
 *             h(t,x(t),u(t),p) <= 0,   hT(T,x(t),p) <= 0
 *             u_min <= u(t) <= u_max
 *             p_min <=  p   <= p_max
 *             T_min <=  T   <= T_max
 *
 *
 */

#include "probfct.h"
#include "math.h"
#include "my_cpp_py_pkg/userparam.h"

#if USE_typeRNum == USE_FLOAT
#define SIN(a)		sinf(a)
#define COS(a)		cosf(a)
#define TAN(a)      tanf(a)
#else
#define SIN(a)		sin(a)
#define COS(a)		cos(a)
#define TAN(a)      tan(a)
#endif

/* square macro */
#define POW2(a) ((a)*(a))

#define PI 3.14159265358979323846

#define obj_x 0.0
#define obj_y 0.0


typeInt getNearestIndex(double current_x, double current_y, const double* ref, int ref_length) {
    int nearest_idx = 0;
    double min_dist = 10000000.0; // Note: Hardcoded

    for (int i = 0; i < ref_length; i++) {
        double x = ref[3 * i];
        double y = ref[3 * i + 1];
        double d = sqrt((x - current_x) * (x - current_x) + (y - current_y) * (y - current_y));
        if (d < min_dist) {
            min_dist = d;
            nearest_idx = i;
        }
    }

    return nearest_idx;
}

typeRNum wrapToPi(const typeRNum number){
    typeRNum wrappedNumber = fmod(number + PI, 2.0*PI);
    if (wrappedNumber < 0) wrappedNumber += 2.0 * PI;
    wrappedNumber -= PI;

    return wrappedNumber;
}

typeRNum distance_squared(ctypeRNum x,ctypeRNum y,ctypeRNum x_ref,ctypeRNum y_ref){
    return POW2(x - x_ref) + POW2(y - y_ref);
}

typeRNum dx_distance_squared(ctypeRNum x, ctypeRNum x_ref){
    return 2 * (x - x_ref);
}

typeRNum dy_distance_squared(ctypeRNum y, ctypeRNum y_ref){
    return 2 * (y - y_ref);
}

typedef struct {
    double x;
    double y;
    double yaw;
} my_point;

my_point calculate_projected_ref_point(double* ref, int ref_length, double* x){

    int nearest_idx = getNearestIndex(x[0], x[1], ref, ref_length);
    int next_idx = (nearest_idx + 1)%ref_length;

    // Coords of nearest point
    double nearest_x = ref[3*nearest_idx];
    double nearest_y = ref[3*nearest_idx + 1];

    // Coords of next point
    double next_x = ref[3*next_idx];
    double next_y = ref[3*next_idx + 1];

    // Vector entries from nearest to current position
    double nearest_to_current_x = x[0] - nearest_x;
    double nearest_to_current_y = x[1] - nearest_y;

    // Normalized vector entries from nearest to next point
    double nearest_to_next_x = next_x - nearest_x;
    double nearest_to_next_y = next_y - nearest_y;

    double nearest_to_next_length = sqrt(POW2(nearest_to_next_x) + POW2(nearest_to_next_y));
    nearest_to_next_x /= nearest_to_next_length;
    nearest_to_next_y /= nearest_to_next_length;

    // Projection of vector nearest_to_current onto nearest_to_next, resulting in the reference x and y
    double proj = (nearest_to_current_x * nearest_to_next_x) + (nearest_to_current_y * nearest_to_next_y); // dot(ntc, ntn)

    double x_ref = nearest_x + proj * nearest_to_next_x;
    double y_ref = nearest_y + proj * nearest_to_next_y;
    double yaw_ref = ref[3*nearest_idx + 2];

    // Result structure
    my_point ref_point;

    ref_point.x = x_ref;
    ref_point.y = y_ref;
    ref_point.yaw = yaw_ref;

    return ref_point;
}


// /** OCP dimensions: states (Nx), controls (Nu), parameters (Np), equalities (Ng), 
//     inequalities (Nh), terminal equalities (NgT), terminal inequalities (NhT) **/
void ocp_dim(typeInt *Nx, typeInt *Nu, typeInt *Np, typeInt *Ng, typeInt *Nh, typeInt *NgT, typeInt *NhT, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;
    //int num_obs = param->num_obstacles;

    //printf("Maximum number of obstacles: %d\n", num_obs);

    *Nx = param->NX;                        //Number of states [x, y, yaw, vx, vy, yaw rate]
    *Nu = param->NU;                        //Number of controls [steering, acceleration]
    *Np = 0;                                //Number of parameters
    *Nh = param->NH;                        //Number of inequalities
    *Ng = 0;                                //Number of equalities
    *NhT = 0;                               //Number of terminal inequalities
    *NgT = 0;                               //Number of terminal equalities
}


/** System function f(t,x,u,p,userparam) 
    ------------------------------------ **/
void ffct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    typeRNum L = param->wheelbase;
    typeRNum m = param->m;          // Mass
    typeRNum lf = param->lf;        // Distance from center of mass to front axle
    typeRNum lr = param->lr;        // Distance from center of mass to rear axle

    typeRNum C_af = param->C_af;    // Lateral cornering stiffness front wheel
    typeRNum C_ar = param->C_ar;    // Lateral cornering stiffness rear wheel

    typeRNum Iz = param->I_z;       // Yaw inertia moment 


    typeRNum yaw = x[2];
    typeRNum vx = x[3];
    typeRNum vy = x[4];
    typeRNum dyaw = x[5];

    typeRNum delta = u[0];
    typeRNum a = u[1];

    // Lateral tire forces
    typeRNum Fyf;
    typeRNum Fyr;
    
    if (vx < param->v_switch) {
        Fyf = 0.0;
        Fyr = 0.0;
    } else {
        Fyf = C_af * (delta - (vy + lf * dyaw)/vx);
        Fyr = - C_ar * (vy - lr * dyaw)/vx;
    }

    out[0] = vx * COS(yaw) - vy * SIN(yaw);                 // dx/dt
    out[1] = vx * SIN(yaw) + vy * COS(yaw);                 // dy/dt 
    out[2] = vx/(lf + lr) * TAN(delta);                     // dyaw/dt
    out[3] = dyaw * vy + a;                                 // dvx/dt
    out[4] = -dyaw * vx + 2/m * (Fyf * COS(delta) + Fyr);   // dvy/dt
    out[5] = 2/Iz * (lf * Fyf - lr * Fyr);                  // dyawrate/dt 


    // printf("yaw=%f \n", yaw);
    // for (size_t i = 0; i < 6; i++)
    // {
    //     printf("out[%d]= %.3f, ", i, out[i]);
    //     printf("x[%d]= %.3f, ", i, x[i]);
    // }
    // printf("\n");
}

/** Jacobian df/dx multiplied by vector vec, i.e. (df/dx)^T*vec or vec^T*(df/dx) **/
void dfdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    typeRNum L = param->wheelbase;
    typeRNum m = param->m;          // Mass
    typeRNum lf = param->lf;        // Distance from center of mass to front axle
    typeRNum lr = param->lr;        // Distance from center of mass to rear axle

    typeRNum C_af = param->C_af;    // Lateral cornering stiffness front wheel
    typeRNum C_ar = param->C_ar;    // Lateral cornering stiffness rear wheel

    typeRNum Iz = param->I_z;       // Yaw inertia moment

    typeRNum yaw = x[2];
    typeRNum vx = x[3];
    typeRNum vy = x[4];
    typeRNum dyaw = x[5];

    typeRNum delta = u[0];
    typeRNum a = u[1];

    // Computed using MATLAB
    if (vx < param->v_switch) {
        // Assume zero lateral tire forces
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = vec[1] * (vx * COS(yaw) - vy * SIN(yaw)) - vec[0] * (vy * COS(yaw) + vx * SIN(yaw));
        out[3] = vec[0] * COS(yaw) + vec[1] * SIN(yaw) - dyaw * vec[4] + (vec[2] * TAN(delta)) / (lf+lr);
        out[4] = dyaw * vec[3] + vec[1] * COS(yaw) - vec[0] * SIN(yaw);
        out[5] = vec[3] * vy - vec[4] * vx;
    } else {
        out[0] = 0.0;
        out[1] = 0.0;
        out[2] = vec[1] * (vx * COS(yaw) - vy * SIN(yaw)) - vec[0] * (vy * COS(yaw) + vx * SIN(yaw));
        out[3] = vec[0] * COS(yaw) + vec[1] * SIN(yaw) - vec[4] * (dyaw - (2*((C_ar * (vy - dyaw*lr)) / POW2(vx) + (C_af * COS(delta) * (vy + dyaw * lf)) / POW2(vx))) / m) + (vec[2] * TAN(delta)) / (lf+lr) + (2*vec[5]*((C_af * lf *(vy + dyaw * lf)) / POW2(vx) - (C_ar * lr * (vy - dyaw * lr)) / POW2(vx))) / Iz;
        out[4] = dyaw * vec[3] + vec[1] * COS(yaw) + vec[0] * SIN(yaw) - (2*vec[5]*((C_af * lf) / vx - (C_ar * lr) / vx)) / Iz - (2*vec[4]*(C_ar / vx + (C_af * COS(delta)) / vx)) / m;
        out[5] = vec[3] * vy - vec[4] * (vx - (2*((C_ar * lr) / vx)) / m) - (2*vec[5]*((C_af * POW2(lf)) / vx + (C_ar * POW2(lr)) / vx)) / Iz;
    }
 
    // for (size_t i = 0; i < 6; i++)
    // {
    //     printf("dfdx: out[%d]= %.3f, ", i, out[i]);
    //     printf("x[%d]= %.3f, ", i, x[i]);
    // }
    // printf("\n");

    //printf("dfdx: yaw=%.3f, v=%3.f, vec0=%.3f, vec1=%.3f, vec2=%.3f \n", x[2], x[3], vec[0], vec[1], vec[2]);
}

/** Jacobian df/du multiplied by vector vec, i.e. (df/du)^T*vec or vec^T*(df/du) **/
void dfdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    typeRNum L = param->wheelbase;
    typeRNum m = param->m;          // Mass
    typeRNum lf = param->lf;        // Distance from center of mass to front axle
    typeRNum lr = param->lr;        // Distance from center of mass to rear axle

    typeRNum C_af = param->C_af;    // Lateral cornering stiffness front wheel
    typeRNum C_ar = param->C_ar;    // Lateral cornering stiffness rear wheel

    typeRNum Iz = param->I_z;       // Yaw inertia moment

    typeRNum yaw = x[2];
    typeRNum vx = x[3];
    typeRNum vy = x[4];
    typeRNum dyaw = x[5];

    typeRNum delta = u[0];
    typeRNum a = u[1];

    // Computed using MATLAB
    if (vx < param->v_switch) {
        out[0] = (vec[2] * vx * (POW2(TAN(delta)) + 1))/(lf+lr);
        out[1] = vec[3];
    } else {
        out[0] = (2*C_af*lf*vec[5]) / Iz - (2*vec[4] * (SIN(delta) * (C_af * delta - (C_af * (vy + dyaw * lf)) / vx) - C_af * COS(delta))) / m + (vec[2] * vx * (POW2(TAN(delta)) + 1)) / (lf+lr);
        out[1] = vec[3];
    }

    // for (size_t i = 0; i < 2; i++)
    // {
    //     printf("dfdu: out[%d]= %.3f, ", i, out[i]);
    //     printf("x[%d]= %.3f, ", i, x[i]);
    // }
    // printf("\n");
}

/** Jacobian df/dp multiplied by vector vec, i.e. (df/dp)^T*vec or vec^T*(df/dp) **/
void dfdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *vec, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}


/** Integral cost l(t,x(t),u(t),p,xdes,udes,userparam) 
    -------------------------------------------------- **/
void lfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    double* ref = param->ref_traj;
    int ref_length = param->ref_length;

    // int nearest_idx = getNearestIndex(x[0], x[1], ref, ref_length);

    // double yaw_ref = ref[3*nearest_idx + 2];

    my_point ref_point = calculate_projected_ref_point(ref, ref_length, x);
    double x_ref = ref_point.x;
    double y_ref = ref_point.y;
    double yaw_ref = ref_point.yaw;

    //printf("Current x: %f, y: %f, v: %f, steering: %f, acc: %f \n", x[0], x[1], x[3], u[0], u[1]);

    out[0] =  param->Q_pos   * (POW2(x[0] - x_ref) + POW2(x[1] - y_ref)) // Q_pos may be 0
            + param->Q_theta * POW2(wrapToPi(x[2] - yaw_ref))
            + param->Q_vel   * POW2((x[3] - param->max_velocity) / param->max_velocity)
            + param->R_steer * POW2(u[0]) 
            + param->R_accel * POW2(u[1]); // R_accel may be 0
            
    // printf("proj: %.3f, x_ref: %.3f, y_ref: %.3f, yaw_ref: %.3f , yaw: %.3f \n", proj,x_ref,y_ref,yaw_ref, x[2]);

}

/** Gradient dl/dx **/
void dldx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    double* ref = param->ref_traj;
    int ref_length = param->ref_length;

    // int nearest_idx = getNearestIndex(x[0], x[1], ref, ref_length);

    // double yaw_ref = ref[3*nearest_idx + 2];

    my_point ref_point = calculate_projected_ref_point(ref, ref_length, x);
    double x_ref = ref_point.x;
    double y_ref = ref_point.y;
    double yaw_ref = ref_point.yaw;

    out[0] = param->Q_pos   * 2.0 * (x[0] - x_ref); // Q_pos may be 0
    out[1] = param->Q_pos   * 2.0 * (x[1] - y_ref); // Q_pos may be 0
    out[2] = param->Q_theta * 2.0 * (wrapToPi(x[2] - yaw_ref));
    out[3] = param->Q_vel   * 2.0 * (x[3] - param->max_velocity) / POW2(param->max_velocity); 
    out[4] = 0; 
    out[5] = 0;

}
/** Gradient dl/du **/
void dldu(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{

    UserParam* param = (UserParam*)userparam;

    out[0] = param->R_steer * 2.0 * u[0];
    out[1] = param->R_accel * 2.0 * u[1]; // R_accel may be 0 
}

/** Gradient dl/dp **/
void dldp(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *xdes, ctypeRNum *udes, typeUSERPARAM *userparam)
{
}


/** Terminal cost V(T,x(T),p,xdes,userparam) 
    ---------------------------------------- **/
void Vfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{ 
    UserParam* param = (UserParam*)userparam;

    double* ref = param->ref_traj;
    int ref_length = param->ref_length;

    double x_ref = ref[3*ref_length];
    double y_ref = ref[3*ref_length +1];
    double yaw_ref = ref[3*ref_length +2];

    // my_point ref_point = calculate_projected_ref_point(ref, ref_length, x);
    // double x_ref = ref_point.x;
    // double y_ref = ref_point.y;
    // double yaw_ref = ref_point.yaw;


    out[0] =  param->Q_pos_T    * (POW2(x[0] - x_ref) + POW2(x[1] - y_ref))
            + param->Q_theta_T  * POW2(wrapToPi((x[2] - yaw_ref)))
            + param->Q_vel_T    * POW2(x[3] - param->max_velocity / param->max_velocity);
    
}
/** Gradient dV/dx **/
void dVdx(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{ 
    UserParam* param = (UserParam*)userparam;

    double* ref = param->ref_traj;
    int ref_length = param->ref_length;

    double x_ref = ref[3*ref_length];
    double y_ref = ref[3*ref_length +1];
    double yaw_ref = ref[3*ref_length +2];

    // my_point ref_point = calculate_projected_ref_point(ref, ref_length, x);
    // double x_ref = ref_point.x;
    // double y_ref = ref_point.y;
    // double yaw_ref = ref_point.yaw;

    out[0] = param->Q_pos_T   * 2.0 * (x[0] - x_ref);
    out[1] = param->Q_pos_T   * 2.0 * (x[1] - y_ref);
    out[2] = param->Q_theta_T * 2.0 * wrapToPi((x[2] - yaw_ref));
    out[3] = param->Q_vel_T   * 2.0 * (x[3] - param->max_velocity) / POW2(param->max_velocity); 

} 
/** Gradient dV/dp **/
void dVdp(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}
/** Gradient dV/dT **/
void dVdT(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *xdes, typeUSERPARAM *userparam)
{
}


/** Equality constraints g(t,x(t),u(t),p,uperparam) = 0 
    --------------------------------------------------- **/
void gfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dx multiplied by vector vec, i.e. (dg/dx)^T*vec or vec^T*(dg/dx) **/
void dgdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/du multiplied by vector vec, i.e. (dg/du)^T*vec or vec^T*(dg/du) **/
void dgdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dg/dp multiplied by vector vec, i.e. (dg/dp)^T*vec or vec^T*(dg/dp) **/
void dgdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Inequality constraints h(t,x(t),u(t),p,uperparam) <= 0 
    ------------------------------------------------------ **/
void hfct(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
    UserParam* param = (UserParam*)userparam;

    double* ref = param->ref_traj;
    int ref_length = param->ref_length;

    my_point ref_point = calculate_projected_ref_point(ref, ref_length, x);
    double x_ref = ref_point.x;
    double y_ref = ref_point.y;

    // Velocity constraints
    out[0] = x[3] - param->max_velocity;    // v <= v_max
    out[1] = -x[3];                         // 0 <= v

    // Centerline constraint
    // Calculate squared distance from current position to projected reference point on the centerline
    double dist_center_sqr = distance_squared(x[0], x[1], x_ref, y_ref);
    double r = param->centerline_dist;

    out[2] = dist_center_sqr - POW2(r); // dist_center_sqr <= r^2

    // if (out[2] > 0) {
    //     printf("DISTANCE VIOLATED, d=%.3f\n", sqrt(dist_center_sqr));
    // }

    /* // Obstacle constraints
    int num_obs = param->num_obstacles;
    int points_per_obs = param->points_per_obstacle;
    double* obs_positions = param->obstacles_pos;
    double* obs_sizes = param->obstacles_size;

    for (int i = 0; i < num_obs; i++) {
        out[param->NH + i] = POW2(obs_sizes[i]) - distance_squared(x[0], x[1], obs_positions[i*points_per_obs*2], obs_positions[i*points_per_obs*2 + 1]);
    } */
}
/** Jacobian dh/dx multiplied by vector vec, i.e. (dh/dx)^T*vec or vec^T*(dg/dx) **/
void dhdx_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{   
    UserParam* param = (UserParam*)userparam;

    double* ref = param->ref_traj;
    int ref_length = param->ref_length;

    my_point ref_point = calculate_projected_ref_point(ref, ref_length, x);
    double x_ref = ref_point.x;
    double y_ref = ref_point.y;

    // Calculate squared distance from current position to projected reference point on the centerline
    double dist_center_sqr = (POW2(x[0] - x_ref) + POW2(x[1] - y_ref));

    out[0] = dx_distance_squared(x[0], x_ref) * vec[2];    
    out[1] = dy_distance_squared(x[1], y_ref) * vec[2];
/* 
    // Obstacle constraints
    int num_obs = param->num_obstacles;
    int points_per_obs = param->points_per_obstacle;
    double* obs_positions = param->obstacles_pos;

    for (int i = 0; i < num_obs; i++) {
        out[0] -= (dx_distance_squared(x[0], obs_positions[i*points_per_obs*2]) * vec[param->NH + i]);
        out[1] -= (dy_distance_squared(x[1], obs_positions[i*points_per_obs*2 +1]) * vec[param->NH + i]); 
    } */

    out[2] = 0;
    out[3] = vec[0] - vec[1];
    out[4] = 0;
    out[5] = 0;
}
/** Jacobian dh/du multiplied by vector vec, i.e. (dh/du)^T*vec or vec^T*(dg/du) **/
void dhdu_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dh/dp multiplied by vector vec, i.e. (dh/dp)^T*vec or vec^T*(dg/dp) **/
void dhdp_vec(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Terminal equality constraints gT(T,x(T),p,uperparam) = 0 
    -------------------------------------------------------- **/
void gTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dx multiplied by vector vec, i.e. (dgT/dx)^T*vec or vec^T*(dgT/dx) **/
void dgTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dp multiplied by vector vec, i.e. (dgT/dp)^T*vec or vec^T*(dgT/dp) **/
void dgTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dgT/dT multiplied by vector vec, i.e. (dgT/dT)^T*vec or vec^T*(dgT/dT) **/
void dgTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Terminal inequality constraints hT(T,x(T),p,uperparam) <= 0 
    ----------------------------------------------------------- **/
void hTfct(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, typeUSERPARAM *userparam)
{
    /* UserParam* param = (UserParam*)userparam;

    // Obstacle constraints
    int num_obs = param->num_obstacles;
    int points_per_obs = param->points_per_obstacle;
    double* obs_positions = param->obstacles_pos;
    double* obs_sizes = param->obstacles_size;

    for (int i = 0; i < num_obs; i++) {
        out[i] = POW2(obs_sizes[i]) - distance_squared(x[0], x[1], obs_positions[i*points_per_obs*2], obs_positions[i*points_per_obs*2 + 1]);
    } */
}
/** Jacobian dhT/dx multiplied by vector vec, i.e. (dhT/dx)^T*vec or vec^T*(dhT/dx) **/
void dhTdx_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
    /* UserParam* param = (UserParam*)userparam;

    out[0] = 0.0;
    out[1] = 0.0;

    // Obstacle constraints
    int num_obs = param->num_obstacles;
    int points_per_obs = param->points_per_obstacle;
    double* obs_positions = param->obstacles_pos;

    for (int i = 0; i < num_obs; i++) {
        out[0] -= (dx_distance_squared(x[0], obs_positions[i*points_per_obs*2]) * vec[i]);
        out[1] -= (dy_distance_squared(x[1], obs_positions[i*points_per_obs*2 +1]) * vec[i]);
    }

    out[2] = 0;
    out[3] = 0;
    out[4] = 0;
    out[5] = 0; */
}
/** Jacobian dhT/dp multiplied by vector vec, i.e. (dhT/dp)^T*vec or vec^T*(dhT/dp) **/
void dhTdp_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}
/** Jacobian dhT/dT multiplied by vector vec, i.e. (dhT/dT)^T*vec or vec^T*(dhT/dT) **/
void dhTdT_vec(typeRNum *out, ctypeRNum T, ctypeRNum *x, ctypeRNum *p, ctypeRNum *vec, typeUSERPARAM *userparam)
{
}


/** Additional functions required for semi-implicit systems 
    M*dx/dt(t) = f(t0+t,x(t),u(t),p) using the solver RODAS,
    see GRAMPC docu for more information
    ------------------------------------------------------- **/
/** Jacobian df/dx in vector form (column-wise) **/
void dfdx(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dx in vector form (column-wise) **/
void dfdxtrans(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian df/dt **/
void dfdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Jacobian d(dH/dx)/dt  **/
void dHdxdt(typeRNum *out, ctypeRNum t, ctypeRNum *x, ctypeRNum *u, ctypeRNum *vec, ctypeRNum *p, typeUSERPARAM *userparam)
{
}
/** Mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mfct(typeRNum *out, typeUSERPARAM *userparam)
{
}
/** Transposed mass matrix in vector form (column-wise, either banded or full matrix) **/
void Mtrans(typeRNum *out, typeUSERPARAM *userparam)
{
}

