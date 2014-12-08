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
        

  
// Packets to be received //    
struct udppacket_control
{
    char CLIENT_HEADER;
    unsigned int control_cmd[3];
}client_packet_control;
    
struct udppacket_bool
{
    char CLIENT_HEADER;
    bool data;
}client_packet_bool;

// packets to be sent //
struct udppacket_DAQ
{
    char SERVER_HEADER;
    float data[32];
}client_packet_DAQ;  
    
struct udppacket_COUNTER
{
    char SERVER_HEADER;
    signed int data[12];
}client_packet_COUNTER;  
 
std::ostream& operator<<(std::ostream& os, const struct udppacket_control & obj)
{
    // write obj to stream
    os << " " << obj.CLIENT_HEADER 
    << " " << obj.control_cmd[0] 
    << " " << obj.control_cmd[1] 
    << " " << obj.control_cmd[2];
    return os; 
}  
    
std::ostream& operator<<(std::ostream& os, const struct udppacket_bool & obj)
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
    << " " << obj.data[3];
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
    initial_state << 0, 0, 0, 0;

    /*  Variables used in Controller */
    int p = -1;
    int i = 1;
    int d = 1;
    VectorXd u(3);
    double initial_control = 0;
            
    /* Variables used in serverudp*/
    int msgcnt = 0;
    char buf[BUFSIZE];
    
    udppacket_DAQ send_packet_DAQ;
    udppacket_COUNTER send_packet_COUNTER;
    char*  buffer_send;
  
    udppacket_control * recv_packet_control;
    udppacket_bool * recv_packet_bool;  
    char recv_buffer[BUFSIZE];    
        
    /* Variables used in Timer*/
    double t, present_time, previous_time ;
    time_t now , previous;
    struct timespec spec;
    
    /*  ** Initialization  ** */    
    plant_model *VSA1axis = new plant_model();
        
    VSA1axis -> setProblemDimension(1);
    VSA1axis -> server_start();
        
    VectorXd previous_state = initial_state;
    VectorXd newstate;
            
    u << initial_control,0,0;
        
    int control_len = 0, control_size, state_len = 0, states_size;
    control_size= u.size();
    states_size = previous_state.size();
            
        
    clock_gettime(CLOCK_REALTIME, &spec);
    now  = spec.tv_sec;
    present_time = round(spec.tv_nsec / 1.0e9);
    previous_time = present_time;
        

    for (;;) 
    {
        //cout << "\n Waiting on port : " <<  SERVICE_PORT << "\n";
    		
        VSA1axis -> server_recv(recv_buffer, BUFSIZE);
    		
        switch(recv_buffer[0])
    	{
    		    
    	    case '0' :
    	    {
    		        
    		    recv_packet_control = (udppacket_control *)recv_buffer;
    		        
                std::cout << "\n  server message received is control unsigned int: " << *recv_packet_control << std::endl;
                break;
            }
    		    
    		    
    		    
            case '1' :
            {
                recv_packet_bool = (udppacket_bool *)recv_buffer;
    		        
                std::cout << "\n  server message received is bool type : " << *recv_packet_bool << std::endl;
                break;
            }
    		    
        }
        
        u << (*recv_packet_control).control_cmd[0], (*recv_packet_control).control_cmd[1],
             (*recv_packet_control).control_cmd[2];
            
        cout << "\n double type control u :" << u;
        clock_gettime(CLOCK_REALTIME, &spec);
        now  = spec.tv_sec;
        present_time = round(spec.tv_nsec / 1.0e9);
            
        t = present_time - previous_time;
            

        newstate = VSA1axis -> integrateRK4(t, previous_state, u, timestep);
                    
        previous_state = newstate;
        cout << "\n newstate[3]\n" << newstate(3);       
        double position_motor = newstate(0);
        double speed_motor = newstate(1);
        double position_link = newstate(2);
        double speed_link = newstate(3);
        double ref_pos = 0.52;
                
        send_packet_DAQ.SERVER_HEADER = '0';
        send_packet_DAQ.data[0] = newstate(0);
        send_packet_DAQ.data[1] = newstate(1);
        send_packet_DAQ.data[2] = newstate(2);        
        send_packet_DAQ.data[3] = newstate(3);       
        buffer_send = (char*)&send_packet_DAQ;
        VSA1axis -> server_send(buffer_send, sizeof(send_packet_DAQ));
        struct udppacket_DAQ *asp = &send_packet_DAQ;
        std::cout << "\n  server message sent DAQ: " << *asp << std::endl;
        std::cout << "time at : " << t;  
    }
}