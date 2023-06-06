#include "car_kin_ode.h"

#include <boost/math/special_functions/sign.hpp>

car_kin_ode::car_kin_ode(double deltaT) : dt(deltaT), t(0.0), state(3), V(0.0), phi(0.0), vehicleParams_set(false)
{
    // state = [ x, y, theta ]

    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

void car_kin_ode::setInitialState(double x0, double y0, double theta0)
{
    // Initial state values
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
}

void car_kin_ode::setVehicleParams(double L)
{
    // Initialize vehicle parameters
    this->L = L;

    vehicleParams_set = true;
}

void car_kin_ode::setReferenceCommands(double velocity, double steer)
{
    V   = velocity;
    phi = steer;
}

void car_kin_ode::integrate()
{
    // Check vehicle parameters are set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&car_kin_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update time and steering
    t += dt;
}

void car_kin_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x     = state[0];
    const double y     = state[1];
    const double theta = state[2];

    // Vehicle equations
    dstate[0] = V*std::cos(theta);    // dx
    dstate[1] = V*std::sin(theta);    // dy
    dstate[2] = V*std::tan(phi)/L;    // dtheta
}
