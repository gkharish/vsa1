#ifndef PAMINIT_H
#define PAMINIT_H

#define pi 3.14; 
double M, B, Patm, Ps, Lo, b, n, r, tau, ps, cns, c;
M =  10.8;  % Load mass (Kg)
B = 13.1;  % Damping coefficient (kg-s)
Patm = 101e3; % Atmospheric pressure (Pa)
Ps = 653e3 ; % Supply pressure (Pa)
Lo = 135e-3; % PAM initallength (m)
b = 163.5e-3; % Thread length (m)
n = 1.04;  % Number of turns of the thread
r = 5e-2;
tau = 0.2;

ps = Ps/tau;
cns = b^2/3 + (2*Lo) - (Lo^2);
c = (r^2)/(4*pi*(n^2));


// Parameter declaration 
// Nominal values
double M_nom, B_nom
M_nom     =  10.8;   % Load mass (Kg)
B_nom     = 13.1;    % Damping coefficient (kg-s)

M = M_nom*(1);
B = B_nom*(1);


K_ref = 0.06;

//Controller parameteres
lambda = 30;     % control bandwidth
G      = 2e5;    %Robust gain
phi    = 1e4;    %Bondary layer width
#endif