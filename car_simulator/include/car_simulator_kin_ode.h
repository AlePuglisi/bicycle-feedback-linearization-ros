#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulator of rear-wheel drive bicycle dynamic model, Fiala tyre model with saturation, with actuator model
// Control inputs: linear velocity, steering position (as input to actuator model)

class car_simulator_kin_ode
{
public:
    car_simulator_kin_ode(double dT);

    void setInitialState(double x0, double y0, double theta0);
    void setVehicleParams(double L);

    void integrate();

    void setReferenceCommand(double velocity, double steer);

    void getPose(double &x, double &y, double &theta) {x = state[0]; y = state[1]; theta = state[2];};
    void getCommand(double &velocity, double &steer) {velocity = V; steer = phi;};
    void getTime(double &time) {time = t;};

private:
    // Simulation and Vehicle variables
    double t,dt;
    double V, phi; // command
    double L;

    bool vehicleParams_set; // Check variable for proper initialization

    state_type state;
    runge_kutta_dopri5 < state_type> stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double dt);
};