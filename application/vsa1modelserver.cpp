// Copyright (c) 2014 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

//#include <native/task.h>
//#include <native/timer.h>
#include <time.h>
#include <string.h>
#include <sstream>

#include <plantmodel.h>

/* Setting Number of Joints or degree of freedom*/
        void plant_model::setProblemDimension (int n)
        {
            nDOF_ = n;
        }
        
        /* Initialization or setting the parameters */
        void plant_model::setParameters (void)
        {
            
            
            length_ = 1.0; // m
            mass_ = 1.0;   // kg
            friction_ = 0.1; // kg/s
            pressure_musclebase_ = 2.5; //bar
           /* M=1; //Mass (kg)
            K = 30000; //stiffness 
            L = 1; //Length of the rod
            g = 9.8;
            I = 1; //Inertia of the primary motor
            J = 1; */ // Inertia of the other m*/

            
        }
        
        /*PAM system dynamics and Compute state derivatives*/
        
        VectorXd plant_model::computeStateDerivative(double time, VectorXd statevector, VectorXd control)
        {
            double c1,c2,c3,c4;
            VectorXd state_derivative(statevector.size());
            
            c1 =  (M*GRAVITY*L/I);
            c2 = K/I;
            c3 = K/J;
            c4 = I*J/K;
            
            
            state_derivative(0) = statevector(1);
            
            state_derivative(1) = -c1*sin(statevector(0)) - c2*( statevector(0) - statevector(2) );
            
            state_derivative(2) = statevector(3);
            
            state_derivative(3) = c3*( statevector(0) - statevector(2) ) + (1/J)*control(0);
            
            
            return state_derivative;
        }
        
        
        /* Numerical Integrator Rungee Kutta */
        VectorXd plant_model::integrateRK4 (double t, VectorXd state, VectorXd u, double h)
        {
            VectorXd st1 = computeStateDerivative (t, state, u);
            
            VectorXd st2 = computeStateDerivative (t + (0.5 * h), state + (0.5 * h * st1), u);
            
            VectorXd st3 = computeStateDerivative (t + (0.5 * h), state + (0.5 * h * st2),  u);
            
            VectorXd st4 = computeStateDerivative (t + h, state + (h * st3), u);

            VectorXd stNew = state + ( (1/6.0) * h * (st1 + 2.0*st2 + 2.0*st3 + st4) );

           cout << "\n" << "integratorRK4 running";
            return (stNew);
        }
        
        
        /* Numerical Integrator Euler */
        VectorXd plant_model::integrateEuler (double t, VectorXd state, VectorXd u, double h)
        {
            VectorXd st = computeStateDerivative (t, state, u);
            
            VectorXd stNew = state + h*st;
            
            
            return (stNew);
        }
        
        
        
        
    int main(void)
    {
        /* variables used in the principal program */
            int whileloop_counter = 0, error_counter = 0;
            int num_min = 1; /**/
            int FLAG = 1;
            int nsig;
            
            /* Variables used in integrator*/
            double timestep = 0.001;          // timestep used for integrstor 1ms;
            
            /*Variables of system dynamics state space*/
            VectorXd initial_state(4); 
            initial_state << 0,
            0,
            0,
            0;
        /*  Variables used in Controller */
            int p = -1;
            int i = 1;
            int d = 1;
            VectorXd u(3);
            double initial_control = 0;
            
        /* Variables used in serverudp*/
        int msgcnt = 0;
        char buf[BUFSIZE];
        
        
        /* Variables used in Timer*/
        double t, present_time, previous_time ;
        time_t now , previous;
        struct timespec spec;
        
        plant_model *VSA1axis = new plant_model();
        
        VSA1axis -> setProblemDimension(1);
        VSA1axis -> server_start();
        
        VectorXd previous_state = initial_state;
        VectorXd newstate;
            
        u << initial_control,0,0;
        
        int control_len = 0, control_size, state_len = 0, states_size;
        control_size= u.size();
        states_size = previous_state.size();
            
        /*create a packet to be sent or received*/
        clock_gettime(CLOCK_REALTIME, &spec);
        now  = spec.tv_sec;
        present_time = round(spec.tv_nsec / 1.0e9);
        previous_time = present_time;
        
        /**/
        cout << "\n Requesting Client to start the server .....";
        VSA1axis -> server_recv(buf);
        cout << "\n Server running .....";
         for (;;) 
         {
    		//cout << "\n Waiting on port : " <<  SERVICE_PORT << "\n";
    		//int receive_len = 0;
    		//wait for receiving all the message i.e all the elements of u vector
    		//if(control_len <= control_size-1)
    		for(control_len = 0; control_len <= control_size-1; control_len++)
    		{
    		    
    		    VSA1axis -> server_recv(buf);
    		    
    		    u(control_len) = atof(buf);
    		    //flush buf
    		    
    		    //control_len++;
    		    //serversend_FLAG = 1;
    		}
            
            
            cout << "buf[0]" << buf[0];
            cout << "buf[1]" << buf[1];
            cout << "buf[2]" << buf[2];
            
            
            //u(0) = atof(buf);//*( ( double*)buf); //atof(buf);
            //u(0) = strtod(buf, &u1);
            //cout << u(0);
            //u(1) = atof(u1);
            //u(2) = atof(u2);
            
            /*u(0) = atof(buf[0]);// *( ( double*)buf);
            u(1) = atof(buf[1]);
            u(1) = atof(buf[2]);*/
            cout << "\n double type control u :" << u;
            clock_gettime(CLOCK_REALTIME, &spec);
            now  = spec.tv_sec;
            present_time = round(spec.tv_nsec / 1.0e9);
            
            t = present_time - previous_time;
            
            if(control_len == control_size)
            {
                newstate = VSA1axis -> integrateRK4(t, previous_state, u, timestep);
                    
                previous_state = newstate;
                
                double position_motor = newstate(0);
                double speed_motor = newstate(1);
                double position_link = newstate(2);
                double speed_link = newstate(3);
                double ref_pos = 0.52;
                
                ostringstream state, state2, state3, state4;
                //state1 << ref_pos; //newstate(0);
                state2 << newstate(1);
                state3 << newstate(2);
                state4 << newstate(3);
                //strcpy(buf, state3.str().c_str());
                //buf = (char*)&position_motor;
                //sprintf(buf, position_motor, speed_motor, position_link, speed_link, msgcnt++);
        		//printf("\n sending response link position : \"%s\"\n", buf);
        		
        		for(state_len = 0; state_len <= states_size -1; state_len++)
        		{
        		    
        		    state << newstate(state_len);
        		    strcpy(buf, state.str().c_str());
        		    VSA1axis -> server_send(buf);
        		    //flush buf
        		    
        		}
        		
        		control_len = 0;
                
            }
            
         }
    }