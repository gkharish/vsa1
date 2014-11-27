// Copyright (c) 2014 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

#ifndef PLANTMODEL_H
#define PLANTMODEL_H

#include "serverudp3.h"
//#include "clientudp3.h"

#include <system-dynamics.h>
#include <integrate-dynamics.h>

#include <Eigen/Core>
#include <math.h>


#define GRAVITY 9.81 
#define M 1
#define K 1000
#define L 1
#define I 1
#define J 1
#define pi 3.14
using namespace std;
using namespace Eigen;

class  plant_model :  public systemdynamics, public integratedynamics,  public ServerUDP   
{
        protected:
            double length_;
            double mass_;
            double friction_;
            float pressure_muscle1_, pressure_muscle2_, pressure_musclebase_;
            
            int nDOF_;
            
        public:
                /// Constructor
                plant_model () : systemdynamics(), integratedynamics(), ServerUDP()
                {
                    
                }
                
                void setProblemDimension (int n);
                void setParameters (void);
                //void setpidcoeff(int p, int i, int d);
                VectorXd computeStateDerivative (double time, VectorXd state, VectorXd control);
                VectorXd integrateRK4 (double time, VectorXd state, VectorXd control, double timeStep);
                VectorXd integrateEuler (double time, VectorXd state, VectorXd control, double timeStep);
                
                //VectorXd getControl (VectorXd statevector, double reference_position, double position);
                
               
        
         
  
    };
    

    
#endif
