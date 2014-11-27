// Copyright (c) 2014 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/






#ifndef INTEGRATEDYNAMICS_H
# define INTEGRATEDYNAMICS_H

# include <iostream>
#include<Eigen/Core>

using namespace Eigen;

        class integratedynamics
        
        {
            public:
                // Abstract class for system dynamics
                virtual VectorXd integrateRK4 (double, VectorXd, VectorXd, double) = 0;
                virtual VectorXd integrateEuler (double, VectorXd, VectorXd, double) = 0;
        };
    
#endif
