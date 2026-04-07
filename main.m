%% Main.m
clear
clc
close all
% stdatmo.m

ttwistor;

%% 2.1
h = 1609.34; %m
%density = stdatmo(h); % NEED FUNCTION MATHWORKS IS CURRENTLY DOWN
aircraft_state = [0; 0; 0; 0; 0; 0; 21; 0; 0; 0; 0; 0];
%aircraft_surfaces = [de da dr dt]; 
aircraft_surfaces = [0; 0; 0; 0];
wind_inertial = [0; 0; 0];

%% AircraftEOM.m Functin
% Inputs:
%   - time
%   - aircraft_state - 12x1 state vector
%   - aircraft_surfaces - 4x1 control surface vector
%   - wind_inertial - 3x1 wind velocity in inertial coordinates
%   - aircraft_parameters - aircraft parameter structure

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
    [aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
end