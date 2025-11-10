close all; clearvars; clc;

%% Charge vehicle parameters
models = jsondecode(fileread("parameters/bicycle.json"));
model = models.bicycle;

% Compute additional parameters
model.L   = model.lr + model.lf;                        % Wheelbase [m]
model.m_f = model.m * model.lr / model.L;               % Front axle mass [kg]
model.m_r = model.m * model.lf / model.L;               % Rear axle mass [kg]
model.Kus = model.m_f/model.Cf - model.m_r/model.Cr;    % Understeer gradient [1/rad]

%% Initial conditions
v0 = 100 / 3.6;        % Initial longitudinal speed [m/s]
x0 = [0; 0];           % Initial lateral speed and yaw rate [m/s]
y0 = 0; yp0 = 0;       % Initial lateral position and its velocity [m], [m/s]

sim("examples/simulator_linearbicycle.slx");