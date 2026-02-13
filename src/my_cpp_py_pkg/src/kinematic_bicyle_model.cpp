
#include <fstream>
#include <sstream>
#include <vector>
#include <tuple>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

class kinematic_bicycle_model{

    struct my_state{
            double x;
            double y;
            double yaw;
            double v;
        };

    struct my_input{
            double a;
            double steer;
        };

    public:
    
        my_state state0;
        my_input input0;

        double t;
        double L;


        void set_state0(double x, double y, double yaw, double v, double a, double steer, double dt, double wheelbase){

            state0.x = x;
            state0.y  = y;
            state0.yaw = yaw;
            state0.v = v;
            input0.a = a;
            input0.steer = steer;
            t = dt;
            L = wheelbase;
        }

        my_state get_next_state(){
            my_state state_t;

            //dx/dt
            double dx = state0.v* cos(state0.yaw);
            state_t.x = state0.x + dx*t;

            //yt = y0 + y' *t;
            double dy = state0.v* sin(state0.yaw);
            state_t.y = state0.y + dy *t;

            //yawt = yaw0 + yaw' *t;
            double dyaw = state0.v * tan(input0.steer)/L;
            state_t.yaw = state0.yaw + dyaw *t;

            //vt = v0 + a *t;
            state_t.v = state0.v + input0.a *t;

            return state_t;

        }



    private:



};
