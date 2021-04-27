%% Include the necessary functions and constants
addpath('./model');
addpath('./Trajectory Generation');
addpath('./Functions');
addpath('./Functions/Quaternion');
RAD2DEG = 180 / pi;
G_acc = 9.81;
%% Initialize the quadrotor model 
mdl_cnstr = vehicle_model();

% Physical parameters
Mass = 0.032;
InertiaMatrix = [
    16.571, 0.830, 0.718;
    0.831, 16.656, 1.800;
    0.718, 1.800, 29.262];
InertiaMatrix = InertiaMatrix * 1e-6;

Inv_InertiaMatrix = inv(InertiaMatrix);

body_arm = 0.04;        % distance between a prop and the centre of mass

Mix_XY = [
    -1, -1, 1,  1;
    -1,  1, 1, -1
    ];
Mix_Z = [1, -1, 1, -1];

% Drags
linear_drag_coef = 5e-7;
quat_drag_coef = 5e-7;
rot_linear_drag_coef = 5e-7;