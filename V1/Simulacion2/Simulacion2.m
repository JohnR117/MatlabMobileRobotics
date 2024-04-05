clear;
%% Parameters
La=0.94; %H, inductance of the armature winding
Ra=1; %Ohm, resistance of the armature winding
Kb= 1;%0.0301; %V/rad/s the back emf constant 

N=1;  %the gear ratio
Kt=0.0300; %Nm/Amp the torque constant

J = 2; % Moment inertia of robot KG*m^2
M = 5; % Robot mass in KG
d = 0.02; % Location of center of gravity of robot. x=d y=o in robot frame
R = 0.02;%radius of wheel in meter
L = 0.1;%distance between wheel in meter


%% Input
V_r = 1; % right motor voltage
V_l = 0; % right motor voltage

G_x = 1; %Goal x position
G_y = -1; %Goal y position
G_g = pi/2; %Goal z orientation

xa=0;
ya=0;
theta=0;

vol_gain = 1; % to slowdown the robot

TF = 40; %simulation time

%% Run simulation
simout = sim('ModeloSimulacion2','StartTime','0','StopTime','40','FixedStep','1.0');
figure(2);
subplot(2,2,1),plot(simout.ya), xlabel('t(s)'), ylabel('ya(m)');
subplot(2,2,2),plot(simout.xa), xlabel('t(s)'), ylabel('xa(m)');
subplot(2,2,3),plot(simout.xa.Data, simout.ya.Data, G_x, G_y, '*'), xlabel('xa(m)'), ylabel('ya(m)');
subplot(2,2,4),plot(simout.theta), xlabel('t(s)'), ylabel('theta(rad)');
%%subplot(3,3,5),plot(simout.tout, simout.poser), xlabel('t(s)'), ylabel('position error(m)');
%%subplot(3,3,6),plot(simout.tout, simout.orrer), xlabel('t(s)'), ylabel('heading error(deg)');
%%subplot(3,3,7),plot(simout.tout, simout.vr_flc), xlabel('t(s)'), ylabel('right motor armature (volt)');
%%subplot(3,3,8),plot(simout.tout, simout.vl_flc), xlabel('t(s)'), ylabel('left motor armature (volt)');
%%subplot(3,3,9),plot(simout.tout, simout.vl_flc, 'r', simout.tout, simout.vr_flc, 'b'), xlabel('t(s)'), ylabel('armature (volt)');