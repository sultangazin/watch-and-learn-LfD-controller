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
% This code collects demonstrations of a controller from:
% D. Mellinger and V. Kumar, 
% "Minimum snap trajectory generation and control for quadrotors," 
% 2011 IEEE International Conference on Robotics and Automation, 2011,
% pp. 2520-2525, doi: 10.1109/ICRA.2011.5980409.
%
% and, based on these demonstrations, constructs a controller.
%
%% Clean the workspace
clear all;
close all;
%% Initialize
init;
%% Expert Controller (Mellinger)
initMKController;
%% Simulate the expert
close all;

% Start
initialPosition = zeros(3,1);
initialVelocity = zeros(3,1);
q0 = [1;0;0;0];

% Initial states for each of experiments/expert demonstrations
initials = eye(9);
initials(3,1) = 1.2;
initials(3,2) = 1.4;
initials(1,4) = 0.1;
initials(2,5) = 0.1;
initials(3,6) = 0.1;
initials(1,7) = 0.1;
initials(2,8) = 0.1;
initials(3,9) = 0.1;

% how big of an operational envelope? 
% (too large and the controller destabilizes)
x_factor = 0.1;
initials = x_factor * initials;

% duration of each demonstration
T = 10;
% sampling time
Ts = 0.001;

for i = 1:9
    % Store a waypoint
    intermediate_z = initials(:,i);
    
    sample_Times = 0:Ts:(T/2);
    % Create a trajectory from the origin to a waypoint
    [polys_1, polys_v_1, polys_a_1] = CreateTrajectory(zeros(9,1), intermediate_z, T/2, sample_Times);  
    % Concatenate the position trajectory with a waypoint to the origin
    polys = [polys_1(:,2:4); zeros(size(sample_Times(2:end),2),3)];
    % Append the time vector
    polys = [(0:Ts:T)', polys];
    % Concatenate the velocity trajectory with a waypoint to zero velocity
    polys_v = [polys_v_1(:,2:4); zeros(size(sample_Times(2:end),2),3)];
    polys_v = [(0:Ts:T)', polys_v];
    % Concatenate the acc trajectory with a waypoint to zero acc
    polys_a = [polys_a_1(:,2:4); zeros(size(sample_Times(2:end),2),3)];
    polys_a = [(0:Ts:T)', polys_a];
    out(i) = sim('expert_sim',T);
end
%% Construct and plot the demonstrations

% construct demonstration matrices (start time from the waypoint)
time = out(1).z.time(5001:end) - min(out(1).z.time(5001:end));
Z = zeros(9,9,size(time,1));    % trajectories
V = zeros(3,9,size(time,1));    % inputs

% store the demonstrations
for i = 1:9
    Z(:,i,:) = out(i).z.signals.values(5001:end,:)';
    V(:,i,:) = out(i).v.signals.values(5001:end,:)';
end

% Calculate the norms and determinants of trajectories
Z_norms = zeros(1,length(time));
dets_Z = zeros(1,length(time));
for t = 1:length(time)
    Z_norms(t) = norm(Z(:,:,t));
    dets_Z(t) = det(Z(:,:,t));
end

% Plot
figure;
plot(time,Z_norms)
ylabel('||Z(t)||');
xlabel('t');
figure;
plot(time,dets_Z)
ylabel('det(Z(t))');
xlabel('t');

% Calculate the norms of inputs
V_norms = zeros(1,length(time));
for t = 1:length(time)
    V_norms(t) = norm(V(:,:,t));
end

% Plot the norms of inputs
figure;
plot(V_norms)
title('Norms of V');

% Construct the "instanteneous" controller
K = zeros(3,9,size(time,1));
for i = 1:length(time)
   K(:,:,i) = V(:,:,i)/Z(:,:,i);
end

%% Create a controller and initialize LfD simulink (for stabilization)

LfD_sim_t = 300.0;  % simulation length
controller.time = 0:Ts:LfD_sim_t;
% window used for controller values
window_start = 0.1;   %s
window_end = 2.5;    %s
window_start_idx = window_start/Ts + 1;
window_end_idx = window_end/Ts + 1;

controller.signals.values = cat(3,K(:,:,window_start_idx:window_end_idx),...
                            repmat(K(:,:,(window_start_idx+1):window_end_idx),1,1,150));
controller.signals.values = controller.signals.values(:,:,1:length(controller.time));
controller.signals.dimensions = [size(controller.signals.values,1),...
                                size(controller.signals.values,2)];

% %% (Debug) Initialize state
% initial_z = 5 * rand(9,1);  % random initial position
% initialPosition = initial_z(1:3);
% initialVelocity = initial_z(4:6);
% initialAcc = initial_z(7:9);
% q0 = [1 0 0 0];

% %% (Debug) Find the largest drop in norm
% search_window = [101,2501];
% minimum = 1;
% indices = [0,0];
% for i = search_window(1):search_window(2)
%     for j = i:search_window(2)
%         if minimum >= norm(Z(:,:,j)/Z(:,:,i))
%             minimum = norm(Z(:,:,j)/Z(:,:,i));
%             indices = [i,j];
%         end
%     end
% end

% %% (Debug) Ideal initial conditions
% initialPosition = [0;0;0];
% initialVelocity = [0;0;0];
% initialAcc = [0;0;0];
% q0 = [1 0 0 0];
% %% (Debug) plot the controller value norms
% 
% ctrl_norms = zeros(1,length(controller.signals.values));
% for t = 1:length(controller.signals.values)
%     ctrl_norms(t) = norm(controller.signals.values(:,:,t));
% end
% 
% figure;
% plot(ctrl_norms)
% title('Norms of controller');
% %% (Debug) Plot K
% K_plot = zeros(3001,27);
% for i = 1:3001
%     K_plot(i,:) = reshape(K(:,:,i),1,[]);
% end