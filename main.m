%% Main.m
clear
clc
close all

% Calls in aircraft parameter
ttwistor;

%% 2.1
initial_state_21 = [0; 0; -1609.34; 0; 0; 0; 21; 0; 0; 0; 0; 0];
%aircraft_surfaces = [de da dr dt]; 
aircraft_surfaces_21 = [0; 0; 0; 0];
wind_inertial = [0; 0; 0];
tspan = [0 10];

[t_out_21, state_out_21] = ode45(@(time, aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces_21, wind_inertial, aircraft_parameters), tspan, initial_state_21);

%% 2.2
initial_state_22 = [0; 0; -1800; 0; 0.02780; 0; 20.99; 0; 0.5837; 0; 0; 0];
aircraft_surfaces_22 = [0.1079; 0; 0; 0.3182];
[t_out_22, state_out_22] = ode45(@(time, aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces_22, wind_inertial, aircraft_parameters), tspan, initial_state_22);

%% 2.3
con2rad = pi/180;
initial_state23 = [0;   0;    -1800; 
                15*con2rad; -12*con2rad; 270*con2rad; 
                19; 3; -2; 
                0.08*con2rad; -0.2*con2rad; 0];
%aircraft_surfaces = [de da dr dt]; 
aircraft_surfaces23 = con2rad*[5; 2; -13; 0.3];

[t_out_23, state_out_23] = ode45(@(time, aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces23, wind_inertial, aircraft_parameters), tspan, initial_state23);