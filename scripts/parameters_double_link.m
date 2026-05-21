clear; clc;

%% Physical parameters
mc = 1.0;      % cart mass, kg

m1 = 0.1;      % link 1 mass, kg
m2 = 0.1;      % link 2 mass, kg

L1 = 0.5;      % link 1 length, m
L2 = 0.5;      % link 2 length, m

lc1 = L1/2;    % distance from joint 1 to center of mass of link 1, m
lc2 = L2/2;    % distance from joint 2 to center of mass of link 2, m

I1 = 0.01;     % moment of inertia of link 1, kg.m^2
I2 = 0.01;     % moment of inertia of link 2, kg.m^2

g = 9.81;      % gravity acceleration, m/s^2

%% Damping parameters

bc = 0.5;      % cart damping or friction coefficient
b1 = 0.001;    % joint 1 damping coefficient
b2 = 0.001;    % joint 2 damping coefficient

%% Input limit

u_max = 15;    % maximum cart force, N

%% Simulation settings

sim_time = 10; % total simulation time, s
dt = 0.01;     % simulation step time, s

%% Initial conditions

theta1_0 = deg2rad(3);     % initial angle of link 1, rad
theta2_0 = deg2rad(-3);    % initial angle of link 2, rad

% State vector:
% X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]
x0 = [0;
      0;
      theta1_0;
      0;
      theta2_0;
      0];

%% Basic checks

if length(x0) ~= 6
    error('Initial state x0 must have 6 elements.');
end

if size(x0, 2) ~= 1
    error('Initial state x0 must be a 6x1 column vector.');
end

if mc <= 0 || m1 <= 0 || m2 <= 0
    error('Mass values must be positive.');
end

if L1 <= 0 || L2 <= 0
    error('Link lengths must be positive.');
end

if lc1 <= 0 || lc2 <= 0
    error('Center of mass distances must be positive.');
end

if I1 <= 0 || I2 <= 0
    error('Moment of inertia values must be positive.');
end

if g <= 0
    error('Gravity must be positive.');
end

if u_max <= 0
    error('Maximum force must be positive.');
end

if sim_time <= 0 || dt <= 0
    error('Simulation time and step time must be positive.');
end

%% Display status

disp('Double link parameters loaded successfully.');
disp('State vector: X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]');
disp('Phase 1 parameter file is ready.');