%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% optimizeTransformationMatrix.m
%
% This script estimates the rigid-body transformation between the camera frame
% and the robot end-effector frame by minimizing the least-squares error
% between corresponding 3D points.
%
% It uses a 6-parameter optimization (Euler angles + translation).
%
% Outputs:
%   - tform_eval (4x4 homogeneous transformation matrix)
%   - TMatrix9.mat
%
% Author: Armando Nicolella, Chiara Cosenza, Sergio Savino,
% LAM4R-University of Naples Federico II
% Last revision: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all          
clear              
clc                

%%
filename = 'test_3_'; % output folder for saving results

% Define transformation function:
% Applies translations and intrinsic rotations around X, Y, Z axes
transformation = @(alfa,beta,gamma,dx,dy,dz) ...
                 makehgtform('translate',[dx dy dz],'xrotate',alfa,'yrotate',beta,'zrotate',gamma);
% alternative commented version using eul2tform + translation separately
% transformation = @(alfa,beta,gamma,dx,dy,dz) eul2tform([alfa,beta,gamma],'XYZ') + makehgtform('translate',[dx dy dz]);

%% Load Data
load('centro_fit.mat') % centers detected in camera frame
load('Ppose.mat')      % centers known in robot frame

%% Define corresponding points
% Points in camera system (centro_fit)
p1 = centro_fit(:,1); 
p2 = centro_fit(:,2); 
p3 = centro_fit(:,3);
p4 = centro_fit(:,4); 
p5 = centro_fit(:,5); 
p6 = centro_fit(:,6);
p7 = centro_fit(:,7); 
p8 = centro_fit(:,8); 
p9 = centro_fit(:,9);

% Points in robot system (Ppose)
p1t = Ppose(:,1); p2t = Ppose(:,2); p3t = Ppose(:,3);
p4t = Ppose(:,4); p5t = Ppose(:,5); p6t = Ppose(:,6);
p7t = Ppose(:,7); p8t = Ppose(:,8); p9t = Ppose(:,9);

%% Calibration setup
xdata = [p1,p2,p3,p4,p5,p6,p7,p8,p9]; % camera points
ydata = [p1t,p2t,p3t,p4t,p5t,p6t,p7t,p8t,p9t]; % robot points

% Initial guess (no rotation, no translation)
x0 = [0 0 0 0 0 0]; 

% Lower and upper bounds
lb = [-pi, -pi, -pi, -Inf, -Inf, -Inf]; % rotation bounds in radians, translation unbounded
ub = [ pi,  pi,  pi,  Inf,  Inf,  Inf];

% Cost function for optimization
fun = @(x,xdata) transformation(x(1),x(2),x(3),x(4),x(5),x(6)) * xdata;

% Solve with least-squares curve fitting
[x_eval, resnorm, residual, exitflag, output, lambda, jacobian] = ...
    lsqcurvefit(fun, x0, xdata, ydata, lb, ub);

% Retrieve optimized transformation
tform_eval = transformation(x_eval(1), x_eval(2), x_eval(3), x_eval(4), x_eval(5), x_eval(6));

%% Save results
save('TMatrix9','tform_eval');       % save optimized matrix
save([filename,'TMatrix9'],'tform_eval'); % save also inside named folder

% Move output files into the specified folder
movefile('*.mat', filename)
