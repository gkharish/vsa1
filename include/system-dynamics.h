// Copyright (c) 2014 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

   
#ifndef SYSTEMDYNAMICS_H
# define SYSTEMDYNAMICS_H
       
#include <iostream>
#include<Eigen/Dense>
#include<Eigen/Core>
   
   
   
   using namespace std;
   using namespace Eigen;
   
   class systemdynamics 
   {
       
       
       public:
                // Abstract class for system dynamics
                virtual void setProblemDimension (int) = 0;
                virtual VectorXd computeStateDerivative (
                        double time, VectorXd state, VectorXd control) = 0;
               // virtual MatrixXd simulateDynamics (VectorXd timeVec,
                        //VectorXd initState) = 0;
                //virtual MatrixXd simulateDynamics (VectorXd timeVec,
                        //VectorXd initState, MatrixXd controlFeedFwd) = 0;
   };
   
   #endif