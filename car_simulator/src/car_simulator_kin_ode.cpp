#include "car_simulator_kin_ode.h"

#include <boost/math/special_functions/sign.hpp>

car_simulator_kin_ode::car_simulator_kin_ode(double dT): dt(dT), t(0.0), state(3), V(0.0), phi(0.0), vehicleParams_set(false)
{
    // state = [x, y, theta]
    // Initial state values
    state[0] = 0.0;
    state[1] = 0.0;
    state[2] = 0.0;
}

void car_simulator_kin_ode::setInitialState(double x0, double y0, double theta0)
{
    // Initial state values set bu user
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;
}

void car_simulator_kin_ode::setVehicleParams(double L)
{
    this->L = L;

    vehicleParams_set = true;
}

void car_simulator_kin_ode::setReferenceCommand(double velocity, double steer)
{
    V = velocity;
    phi = steer;
}

void car_simulator_kin_ode::integrate()
{
    // Check correct vehicle parameter set
    if (!vehicleParams_set) {
        throw std::invalid_argument( "Vehicle parameters not set!");
    }

    // one step integration
    using namespace std::placeholders;
    stepper.do_step(std::bind(&car_simulator_kin_ode::vehicle_ode, this, _1, _2, _3), state, t, dt);

    // Update integration time
    t += dt;
}

void car_simulator_kin_ode::vehicle_ode(const state_type &state, state_type &dstate, double t)
{
    using namespace boost::math;

    // Actual state
    const double x    = state[0];
    const double y = state[1];
    const double theta    = state[2];


    // Vehicle equations, state space model
    dstate[0] = V*std::cos(theta);                       // dx
    dstate[1] = V*std::sin(theta);                       // dy
    dstate[2] = V*std::tan(phi)/L;                       // dtheta
}  


