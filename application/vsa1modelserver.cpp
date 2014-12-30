// Copyright (c) 2014 CNRS
// Authors: Ganesh Kumar


/* ******** It is a trial program for modelling a Pneumatic muscle and 
   ******** controlling it in closeloop under xenomai realtime kernel ********/

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>

#include <native/task.h>
#include <native/timer.h>
//#include <native/task.h>
//#include <native/timer.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <math.h>
#include <plantmodel.h>
//#include <paminit.h>
/* Setting Number of Joints or degree of freedom*/
void plant_model::setProblemDimension (int n)
{
    nDOF_ = n;
}
        
/* Initialization or setting the parameters */
void plant_model::setParameters (void)
{
    M =  10.8;                      // Load mass (Kg)
    B = 13.1;                       // Damping coefficient (kg-s)
    Patm = 101e3;                   // Atmospheric pressure (Pa)
    Ps = 653e3 ;                    // Supply pressure (Pa)
    Lo = 135e-3;                    // PAM initallength (m)
    b = 163.5e-3;                   // Thread length (m)
    n = 1.04;                       // Number of turns of the thread
    r = 5e-2;
    tau = 0.2;

    ps = Ps/tau;
    cns = pow(b, 2)/3 + (2*Lo) - pow(Lo, 2);
    c = pow(r, 2)/(4*pi*pow(n, 2));


    // Parameter declaration 
    // Nominal values

    M_nom     =  10.8;              // Load mass (Kg)
    B_nom     = 13.1;               // Damping coefficient (kg-s)

    M = M_nom*(1);
    B = B_nom*(1);


    K_ref = 0.06;

    //Controller parameteres
    lambda = 30;                    // control bandwidth
    G      = 2e5;                   // Robust gain
    phi    = 1e4;                   // Bondary layer width
         
            
   
}
        
 /*PAM system dynamics and Compute state derivatives*/
        
VectorXd plant_model::computeStateDerivative(double time, VectorXd statevector, VectorXd control)
{
    
    VectorXd state_derivative(statevector.size());       
    //states = [x, x_dot, x_dot2 , Pa, Pb]
    //subequations
    double tb, ta, den;
    tb = 3*(pow( (Lo - statevector(0)), 2) ) - pow(b, 2);
    ta = 3*( pow( (Lo + statevector(0)), 2) ) - pow(b, 2);
    den = 4*pi*pow(n, 2)*M;

    state_derivative(0) =  statevector(1);

    state_derivative(1) =  statevector(2); //((tb - ta)*(-Patm)/den) - ( (B/M)*x(2) ) + (tb*x(5)/den) - (ta*x(4)/den);

    state_derivative(3)  =  (-statevector(3)/tau) + (Ps/tau)*control(0);
    state_derivative(4)  =  (-statevector(4)/tau) + (Ps/tau)*control(1);

    state_derivative(2) = ((tb/den)*state_derivative(4)) - ((ta/den)*state_derivative(3)) 
                        - ( ( 3/(2*pi*pow(n, 2)*M) )*(  (Lo - statevector(0))*(statevector(4)-Patm) + 
                            (Lo +statevector(0))*(statevector(3)-Patm)  )*statevector(1)  ) - ((B/M)*statevector(2));
        
    return state_derivative;
}
        
double plant_model::computeStiffness(VectorXd statevector, VectorXd control)
{
    
    den = 4*pi*pow(n, 2)*M;
    sigma1 =  r*r*(M/den) *(  ((statevector(3) - Patm)*6*(Lo +statevector(0)) ) + 
                ((statevector(4) - Patm)*6*(Lo - statevector(0)) )  );
    
    return sigma1;
}
/* Numerical Integrator Rungee Kutta */
VectorXd plant_model::integrateRK4 (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st1 = computeStateDerivative (t, state, u);
            
    VectorXd st2 = computeStateDerivative (t + (0.5 * h), state + (0.5 * h * st1), u);
            
    VectorXd st3 = computeStateDerivative (t + (0.5 * h), state + (0.5 * h * st2),  u);
            
    VectorXd st4 = computeStateDerivative (t + h, state + (h * st3), u);

    VectorXd stNew = state + ( (1/6.0) * h * (st1 + 2.0*st2 + 2.0*st3 + st4) );

    
    return (stNew);
}
        
        
/* Numerical Integrator Euler */
VectorXd plant_model::integrateEuler (double t, VectorXd state, VectorXd u, double h)
{
    VectorXd st = computeStateDerivative (t, state, u);
            
    VectorXd stNew = state + h*st;
    return (stNew);
}
        

  
// Packets to be received //    
struct udppacket_control                    // clientheader = '0';
{
    char CLIENT_HEADER;
    double control_cmd[3];
    //unsigned int control_cmd[16];
}client_packet_control;
    
struct udppacket_countersreset              // clientheader = '1';
{
    char CLIENT_HEADER;
    bool data;
}client_packet_countersreset;

struct udppacket_digitaloutputcontrol        // clientheader = '2';
{
    char CLIENT_HEADER;
    bool data;
}client_packet_digitaloutputcontrol;

// packets to be sent //
struct udppacket_DAQ
{
    char SERVER_HEADER;                     // serverheader = 'a';
    float data[32];
}client_packet_DAQ;  
    
struct udppacket_COUNTER
{
    char SERVER_HEADER;                     // serverheader = 'b';
    signed int data[12];
}client_packet_COUNTER;  

struct udppacket_error                      // serverheader = 'c';
{
    char SERVER_HEADER;
    unsigned char data[4];
}client_packet_error; 


std::ostream& operator<<(std::ostream& os, const struct udppacket_control & obj)
{
    // write obj to stream
     os << " " << obj.CLIENT_HEADER 
	<< " " << obj.control_cmd[0] 
	<< " " << obj.control_cmd[1] 
	<< " " << obj.control_cmd[2];
    return os; 
}  
    
std::ostream& operator<<(std::ostream& os, const struct udppacket_countersreset & obj)
{
    // write obj to stream
    os << " " << obj.CLIENT_HEADER 
	<< " " << obj.data; 
	 
    return os; 
}  

std::ostream& operator<<(std::ostream& os, const struct udppacket_digitaloutputcontrol & obj)
{
    // write obj to stream
    os << " " << obj.CLIENT_HEADER 
	<< " " << obj.data; 
	 
    return os; 
}

std::ostream& operator<<(std::ostream& os, const struct udppacket_DAQ & obj)
{
    // write obj to stream
    os << " " << obj.SERVER_HEADER 
    << " " << obj.data[0] 
    << " " << obj.data[1] 
    << " " << obj.data[2]
    << " " << obj.data[3]
    << " " << obj.data[4];
    return os; 
}  
    
std::ostream& operator<<(std::ostream& os, const struct udppacket_COUNTER & obj)
{
    // write obj to stream
    os << " " << obj.SERVER_HEADER 
    << " " << obj.data[0] 
    << " " << obj.data[1]; 
    return os; 
}        
        
std::ostream& operator<<(std::ostream& os, const struct udppacket_error & obj)
{
    // write obj to stream
    os << " " << obj.SERVER_HEADER 
    << " " << obj.data[0] 
    << " " << obj.data[1]
    << " " << obj.data[2]
    << " " << obj.data[3];
    return os; 
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
    VectorXd initial_state(5); 
    initial_state << 0.01, 0, 0, 101e3, 101e3;

    /*  Variables used in Controller */
    int p = -1;
    int i = 1;
    int d = 1;
    VectorXd u(3);
    u << 0.1 , 0, 0;
    
    /*  Variables used in real time Timer   */     
    RTIME  now, previous, TASK_PERIOD = 1000000;
    double t, time_start_loop, present_time;
    
    /* Variables used in serverudp*/
    
    
    udppacket_DAQ send_packet_DAQ;
    udppacket_COUNTER send_packet_COUNTER;
    udppacket_error send_packet_error;
    char*  buffer_send;
  
    udppacket_control * recv_packet_control;
    udppacket_countersreset * recv_packet_countersreset;
    udppacket_digitaloutputcontrol * recv_packet_digitaloutputcontrol;
    char recv_buffer[BUFSIZE];    
        
    /* Variables used in Timer*/
    /*double t, present_time, previous_time ;
    time_t now , previous;
    struct timespec spec;*/
    
    /*  ** Initialization  ** */    
    plant_model *PAM1axis = new plant_model();
        
    PAM1axis -> setProblemDimension(1);
    PAM1axis -> setParameters();
    PAM1axis -> server_start();
        
    VectorXd previous_state = initial_state;
    VectorXd newstate;
            
    
        
    int control_len = 0, control_size, state_len = 0, states_size;
    control_size= u.size();
    states_size = previous_state.size();
            
        
    /*clock_gettime(CLOCK_REALTIME, &spec);
    now  = spec.tv_sec;
    present_time = round(spec.tv_nsec / 1.0e9);
    previous_time = present_time;*/
    now = rt_timer_read();
    time_start_loop  = round(now/1.0e9);
    int timer_set = 0;   

    for (;;) 
    {
        
        PAM1axis -> server_recv(recv_buffer, BUFSIZE);
    		
        switch(recv_buffer[0])
    	{
    		    
    	    case '0' :
    	    {
    		        
    		    recv_packet_control = (udppacket_control *)recv_buffer;
    		    cout << "\n recv_packet_control \n" << *recv_packet_control;    
                now = rt_timer_read();
                present_time  = round(now/1.0e9);
                t = present_time - time_start_loop;    
                u << (*recv_packet_control).control_cmd[0], (*recv_packet_control).control_cmd[1], 0;
                newstate = PAM1axis -> integrateRK4(t, previous_state, u, timestep);
                
                send_packet_DAQ.SERVER_HEADER = 'a';
                send_packet_DAQ.data[0] = newstate(0);
                send_packet_DAQ.data[1] = newstate(1);
                send_packet_DAQ.data[2] = newstate(2);        
                send_packet_DAQ.data[3] = newstate(3); 
                send_packet_DAQ.data[4] = newstate(4);
                
                
                buffer_send = (char*)&send_packet_DAQ;
                PAM1axis -> server_send(buffer_send, sizeof(send_packet_DAQ));
                struct udppacket_DAQ *asp = &send_packet_DAQ;
                std::cout << "\n  server message sent DAQ: " << *asp << std::endl;
                previous_state = newstate;
                break;
            }
    		    
    		    
            case '1' :
            {
                recv_packet_countersreset = (udppacket_countersreset *)recv_buffer;
    		    cout << "\n recv_packet_countersreset \n" << *recv_packet_countersreset;   
                send_packet_COUNTER.SERVER_HEADER = 'b';
                send_packet_COUNTER.data[0] = 1;
                send_packet_COUNTER.data[1] = 2;
                buffer_send = (char*)&send_packet_COUNTER;
                PAM1axis -> server_send(buffer_send, sizeof(send_packet_COUNTER));
                struct udppacket_COUNTER *asp_COUNTER = &send_packet_COUNTER;
                std::cout << "\n  server message sent COUNTER: " << *asp_COUNTER << std::endl;
                
                break;
            }
            
            case '2' :
            {
                recv_packet_digitaloutputcontrol = (udppacket_digitaloutputcontrol *)recv_buffer;
    		    cout << "\n recv_packet_digitaloutputcontrol \n" << *recv_packet_digitaloutputcontrol;   
                send_packet_error.SERVER_HEADER = 'c';
                send_packet_error.data[0] = 0;
                send_packet_error.data[1] = 0;
                send_packet_error.data[2] = 0;
                send_packet_error.data[3] = 0;
                buffer_send = (char*)&send_packet_COUNTER;
                PAM1axis -> server_send(buffer_send, sizeof(send_packet_error));
                struct udppacket_error *asp_error = &send_packet_error;
                std::cout << "\n  server message sent error: " << *asp_error << std::endl;
                
                break;
            }
    		    
        }
        
        
             
            
        //cout << "\n double type control u :" << u;
        /*clock_gettime(CLOCK_REALTIME, &spec);
        now  = spec.tv_sec;
        present_time = round(spec.tv_nsec / 1.0e9);
            
        t = present_time - previous_time;*/
        
        
                    
        
                

        std::cout << "time at : " << t;  
    }
}