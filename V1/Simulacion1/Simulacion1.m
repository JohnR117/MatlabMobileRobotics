clc;
clear;
%% Parameters
La=3; %H, inductance of the armature winding
Ra=6; %Ohm, resistance of the armature winding
Kb=3; %V/rad/s the back emf constant 

N=2;  %the gear ratio
Kt=2; %Nm/Amp the torque constant

J = 2; % Moment inertia of robot KG*m^2
M = 5; % Robot mass in KG
d = 0.02; % Location of center of gravity of robot. x=d y=o in robot frame
R = 0.02;%radius of wheel in meter
L = 0.1;%distance between wheel in meter


%% Input
V_r = 0; % right motor voltage
V_l = 7; % right motor voltage

%% Run simulation
%%simout = sim('ModeloSimulacion1');
simout = sim('ModeloSimulacion1','StartTime','0','StopTime','20','FixedStep','1.0');
figure(2);
subplot(2,2,1),plot(simout.ya), xlabel('t(s)'), ylabel('ya(m)');
subplot(2,2,2),plot(simout.xa), xlabel('t(s)'), ylabel('xa(m)');
subplot(2,2,3),plot(simout.xa.Data, simout.ya.Data), xlabel('xa(m)'), ylabel('ya(m)');
subplot(2,2,4),plot(simout.theta), xlabel('t(s)'), ylabel('theta(rad)');