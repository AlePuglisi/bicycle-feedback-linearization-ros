#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulator of rear-wheel drive bicycle dynamic model, Fiala tyre model with saturation, with actuator model
// Control inputs: linear velocity, steering position (as input to actuator model)

class car_simulator_dyn_ode
{
public:
    typedef enum e_mode{
        LINEAR = 0,
        FIALA_SATURATION = 1,
        FIALA_NO_SATURATION = 2
    } carModel;

    car_simulator_dyn_ode(double dT, carModel car_model);

    void setInitialState(double r0, double Beta0, double x0, double y0, double psi0);
    void setVehicleParams(double m, double a, double b, double Cf, double Cr, double mu, double Iz);

    void integrate();

    void setReferenceCommand(double velocity, double steer);

    void getPose(double &x, double &y, double &psi) {x = state[2]; y = state[3]; psi = state[4];};
    void getLateralDynamics(double &ay, double &yawrate, double &vy) { ay = this->ay; yawrate = state[0]; vy = state[1];};
    void getSideSlip(double &sideslip) {sideslip = state[1];};
    void getSlip(double &slip_front, double &slip_rear) { slip_front = alphaf; slip_rear = alphar;};
    void getLateralForce(double &force_front, double &force_rear) {force_front = Fyf; force_rear = Fyr;};
    void getCommand(double &velocity, double &steer) {velocity = Vx; steer = delta;};
    void getTime(double &time) {time = t;};

private:
    // Simulation and Vehicle variables
    double t,dt;
    double Vx, delta; // Actuator command
    double sideslip, ay, alphaf, alphar, Fyf, Fyr;
    double m, a, b, Cf, Cr, mu, Iz;

    bool vehicleParams_set; // Check variable for proper initialization
    carModel car_model;
    
    state_type state;
    runge_kutta_dopri5 < state_type> stepper;

    // ODE function
    void vehicle_ode(const state_type &state, state_type &dstate, double dt);
};