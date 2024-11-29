// class containing all the methods to implement the feedback linearization controller
// ( so the inner loop that allows to control a simplifyed system as a couple of indipendent integrator)
class car_fblin
{
    private:
        double P_dist,L;
        double xp, yp;
        double x,y,theta;
    
    
    public:
        car_fblin(double P_dist, double L);

    
        void setState(double x_in, double y_in, double theta_in){ x = x_in; y = y_in; theta = theta_in;};
        
        void commandTransform(double Vxp, double Vyp, double& V, double& phi);
        void stateTransform(double& xp, double& yp);
        void referenceTransform(double xref, double yref, double& xpref, double& ypref);

};