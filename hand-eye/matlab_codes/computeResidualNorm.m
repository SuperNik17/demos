%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% computeResidualNorm.m
%
% This script calculates the residual error between the transformed camera
% points and the corresponding robot-measured points after calibration.
% It computes both the per-axis residuals and the overall residual norm.
%
% Outputs:
%   - residual: matrix containing the error components (X, Y, Z) for each pose
%   - resnormal: total residual norm (scalar)
%
% Author: Armando Nicolella, Chiara Cosenza, LAM4R-University of Napoli
% Federico II
% Last revision: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup
clear
clc

% Load calibration results
load('centro_fit.mat')          % camera-detected marker centers
load('Ppose.mat')               % robot-measured marker centers
load('MatriceTrasformazione9.mat') % optimized transformation matrix

%% Transform camera points into robot frame
residual = zeros(4,9);           % preallocate residual matrix

for i = 1:size(centro_fit,2)     % loop over the 9 poses
    P_camera = centro_fit(:,i);  % select point in camera frame
    P_robot = tform_eval * P_camera; % transform point into robot frame
    P{1,i} = P_robot;            % store transformed point
end 

%% Compute residuals between transformed and measured points
for j = 1:size(P,2)
    diffx = P{j}(1,1) - Ppose(1,j); % X axis residual
    diffy = P{j}(2,1) - Ppose(2,j); % Y axis residual
    diffz = P{j}(3,1) - Ppose(3,j); % Z axis residual
    diff = [diffx, diffy, diffz];   % residual vector
    residuo{1,j} = diff;            % store residual vector

    residual(1,j) = residuo{1,j}(1,1); % save residual X
    residual(2,j) = residuo{1,j}(1,2); % save residual Y
    residual(3,j) = residuo{1,j}(1,3); % save residual Z
end 

%% Compute residual norms
resnorm = sum((residual).^2);    % squared error for each pose
resnormal = sum(resnorm);        % total residual norm
