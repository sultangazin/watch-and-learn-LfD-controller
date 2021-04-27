%% Authors: Alimzhan Sultangazin
% Copyright (C) 2020, Alimzhan Sultangazin
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
% See the GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program. If not, see <http://www.gnu.org/licenses/>.
%
%
% This code is part of the watch-and-learn-LfD-controller repository,
% and is publicly available at: 
% https://github.com/cyphylab/watch-and-learn-LfD-controller.
%
% For any comments contact Alimzhan Sultangazin at asultangazin@g.ucla.edu.
%
%% Description:
% This code simulates the controller based on expert demonstrations
% and commands it to follow a reference trajectory
%% Run this script to collect expert demonstrations and construct controllers
collectExpertDemonstrations;
%% Tune internal PID controller
K_omega_P = 0.0121;
K_omega_I = 0;
K_omega_D = -1.21e-4;
%% Yaw controller
K_yaw = 10;
%% Initial conditions
initialPosition = [0;0;0];
initialVelocity = [0;0;0];
initialAcc = [0;0;0];
q0 = [1 0 0 0];

%% Create a reference
close all;

follow_circ_t = 60.0;  % length of simulations
Approach_t = 5;         % approach time
setpoint_t = 30;        % setpoint time
total_sim_t = 2*Approach_t + follow_circ_t + setpoint_t;
Ts = 0.001;             % sampling time

% Create a circular portion of the reference
follow_circ_t_s = Approach_t + (Ts:Ts:follow_circ_t); % time_idx
a1 = 0.9659; a2 = -2.1560; a3 = -0.2588; a4 = 2.0959;   % coeffs
x_r = a1*cos(2*pi*(follow_circ_t_s-Approach_t)/5) + a2; 
y_r = sin(2*pi*(follow_circ_t_s-Approach_t)/5);
z_r = a3*cos(2*pi*(follow_circ_t_s-Approach_t)/5) + a4;
Reference = [follow_circ_t_s',x_r', y_r', z_r'];

% Calculate the derivatives of the circ. portion
xd_r = gradient(x_r,Ts);
yd_r = gradient(y_r,Ts);
zd_r = gradient(z_r,Ts);
Reference_d = [follow_circ_t_s',xd_r',yd_r',zd_r'];

xdd_r = gradient(xd_r,Ts);
ydd_r = gradient(yd_r,Ts);
zdd_r = gradient(zd_r,Ts);
Reference_dd = [follow_circ_t_s',xdd_r',ydd_r',zdd_r'];

xddd_r = gradient(xdd_r,Ts);
yddd_r = gradient(ydd_r,Ts);
zddd_r = gradient(zdd_r,Ts);
Reference_ddd = [follow_circ_t_s',xddd_r',yddd_r',zddd_r'];

% Create the approach and detach minimum snap trajs
[TrajApproachPos,TrajApproachVel,TrajApproachAcc, TrajApproachJerk]=...
    CreateTrajectory(zeros(9,1), ...
    [Reference(1,2:end) Reference_d(1,2:end) Reference_dd(1,2:end)]', ...
    Approach_t, ...
    0:Ts:Approach_t);
[TrajDetachPos,TrajDetachVel,TrajDetachAcc, TrajDetachJerk]=...
    CreateTrajectory([Reference(1,2:end) Reference_d(1,2:end)...
                        Reference_dd(1,2:end)]',...
    zeros(9,1), Approach_t, 0:Ts:Approach_t);

% offset the time for returning trajectory
TrajDetachPos(:,1) = Approach_t + follow_circ_t + TrajDetachPos(:,1);
TrajDetachVel(:,1) = Approach_t + follow_circ_t + TrajDetachVel(:,1);
TrajDetachAcc(:,1) = Approach_t + follow_circ_t + TrajDetachAcc(:,1);
TrajDetachJerk(:,1) = Approach_t + follow_circ_t + TrajDetachJerk(:,1);

% setpoint reference
setpoint_int = (follow_circ_t + 2*Approach_t):Ts:total_sim_t;
SetPoint = [setpoint_int', zeros(size(setpoint_int,2),3)];

% concatenate all the references
Reference=[TrajApproachPos;Reference;TrajDetachPos(2:end,:);SetPoint(2:end,:)];
Reference_d=[TrajApproachVel;Reference_d;TrajDetachVel(2:end,:);SetPoint(2:end,:)];
Reference_dd=[TrajApproachAcc;Reference_dd;TrajDetachAcc(2:end,:);SetPoint(2:end,:)];
Reference_ddd = [TrajApproachJerk;Reference_ddd;TrajDetachJerk(2:end,:);SetPoint(2:end,:)];

% simulate
out = sim('LfD_sim_ref');

%% Plot the 3D trajectory

figure;
plot3([TrajApproachPos(:,2);x_r(2:end,2);TrajDetachPos(2:end,2)],...
       [TrajApproachPos(:,3);x_r(2:end,3);TrajDetachPos(2:end,3)],...
       [TrajApproachPos(:,4);x_r(2:end,4);TrajDetachPos(2:end,4)],...
       'r--');
axis equal
hold on
plot3(out.z.signals.values(:,1),...
      out.z.signals.values(:,2),...
      out.z.signals.values(:,3),'b');
xlabel('x')
ylabel('y')
zlabel('z')
legend('Reference','Learned controller')

%% Plot the references by component

figure;
subplot(3,1,1);
plot(out.z.time, Reference(:,2), 'r--', out.z.time, out.z.signals.values(:,1),'b')
legend('x_r(t)','x(t)')
xlabel('t');
ylabel('x');
subplot(3,1,2);
plot(out.z.time, Reference(:,3), 'r--', out.z.time, out.z.signals.values(:,2),'b')
legend('y_r(t)','y(t)')
xlabel('t');
ylabel('y');
subplot(3,1,3);
plot(out.z.time, Reference(:,4), 'r--', out.z.time, out.z.signals.values(:,3),'b')
legend('z_r(t)','z(t)')
xlabel('t');
ylabel('z');

%% Plot errors by component
figure;
subplot(3,1,1);
plot(out.z.time, abs(out.z.signals.values(:,1) - Reference(:,2)),'b')
xlabel('t');
ylabel('x-x_r');
subplot(3,1,2);
plot(out.z.time, abs(out.z.signals.values(:,2) - Reference(:,3)),'b')
xlabel('t');
ylabel('y-y_r');
subplot(3,1,3);
plot(out.z.time, abs(out.z.signals.values(:,3) - Reference(:,4)),'b')
xlabel('t');
ylabel('z-z_r');