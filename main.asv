%% Main.m
clear
clc
close all
% stdatmo.m

ttwistor;

%% 2.1
initial_state = [0; 0; 1609.34; 0; 0; 0; 21; 0; 0; 0; 0; 0];
%aircraft_surfaces = [de da dr dt]; 
aircraft_surfaces = [0; 0; 0; 0];
wind_inertial = [0; 0; 0];
tspan = [0 10];

[t_out_21, state_out_21] = ode45(@(time, aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters), tspan, initial_state);

%% AircraftEOM.m Function
% Inputs:
%   - time
%   - aircraft_state - 12x1 state vector
%   - aircraft_surfaces - 4x1 control surface vector
%   - wind_inertial - 3x1 wind velocity in inertial coordinates
%   - aircraft_parameters - aircraft parameter structure

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
% unpack state vector
x = aircraft_state(1); 
y = aircraft_state(2); 
z = aircraft_state(3);
phi = aircraft_state(4); 
theta = aircraft_state(5); 
psi = aircraft_state(6);
u = aircraft_state(7); 
v = aircraft_state(8); 
w = aircraft_state(9);
p = aircraft_state(10); 
q = aircraft_state(11); 
r = aircraft_state(12);

g = aircraft_parameters.g;
m = aircraft_parameters.m;
density = stdatmo(z);
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);


% rotation matrix from body to inertial (ZYX)
R_EB = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
     -sin(theta),          sin(phi)*cos(theta),                             cos(phi)*cos(theta)];
% Position derivative
Pos_dot = R_EB * [u; v; w];
% Euler angle derivative
Euler_dot = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
                      0, cos(phi),           -sin(phi);
                      0, sin(phi)/cos(theta), cos(phi)/cos(theta)] * [p; q; r];
% Velocity derivative
Vel_dot = [r*v-q*w; p*w-r*u; q*u - p*v]+ g*[-sin(theta); cos(theta)*sin(phi); ...
    cos(theta)*cos(phi)] + (1/m)*([aero_forces(1); aero_forces(2); aero_forces(3)]);

Ix = aircraft_parameters.Ix;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;

Gamma = Ix*Iz - Ixz^2;
Gamma1 = (Ixz*(Ix-Iy+Iz))/Gamma;
Gamma2 = (Iz*(Iz-Iy)+Ixz^2)/Gamma;
Gamma3 = Iz/Gamma;
Gamma4 = Ixz/Gamma;
Gamma5 = (Iz-Ix)/Iy;
Gamma6 = Ixz/Iy;
Gamma7 = (Ix*(Ix-Iy)+Ixz^2)/Gamma;
Gamma8 = Ix/Gamma;

% Euler angle rate derivative
Omega_dot = [Gamma1*p*q-Gamma2*q*r+Gamma3*aero_moments(1)+Gamma4*aero_moments(3);
             Gamma5*p*r-Gamma6*((p^2)-(r^2))+(aero_moments(2)/Iy);
             Gamma7*p*q-Gamma1*q*r+Gamma4*aero_moments(1)+Gamma8*aero_moments(3)];

xdot = [Pos_dot; Euler_dot; Vel_dot; Omega_dot];
end