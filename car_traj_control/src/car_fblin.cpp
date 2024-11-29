#include "car_traj_control/car_fblin.h"
#include <cmath>

car_fblin::car_fblin(double P_dist, double L)
{
    this->P_dist  = P_dist;
    this->L = L;
}

// transformation of input Vxp, Vyp to the real vehicle command
void car_fblin::commandTransform( double Vxp, double Vyp, double& V, double& phi)
{
    V = Vxp*std::cos(theta)+Vyp*std::sin(theta);

    // manage the singularity for V = 0:
    if (std::fabs(V)>0.01)
    {
        phi = std::atan(L/(V*P_dist)*(Vyp*std::cos(theta)-Vxp*std::sin(theta)));
    }
    else
    {
        phi = std::atan(L/(0.01*P_dist)*(Vyp*std::cos(theta)-Vxp*std::sin(theta)));
    }
};

// transform real vehciel state into point P cohordinate
void car_fblin::stateTransform(double& xp, double& yp)
{
    xp = x + P_dist*std::cos(theta);
    yp = y + P_dist*std::sin(theta);
}

// transform real reference cohordinate into point P cohordinate
void car_fblin::referenceTransform(double xref, double yref, double& xpref, double& ypref)
{
    xpref = xref + P_dist*std::cos(theta);
    ypref = yref + P_dist*std::sin(theta);
}