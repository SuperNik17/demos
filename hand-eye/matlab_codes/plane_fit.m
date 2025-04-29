%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plane_fit.m
%
% This function fits a plane to a set of 3D points by minimizing the sum 
% of point-to-plane distances. 
% Optionally, a normal vector can be provided to constrain the plane orientation.
%
% Inputs:
%   - pts: [Mx3] matrix of 3D points
%   - n (optional): normal vector to constrain the plane
%
% Output:
%   - model: fitted plane model as a planeModel object
%
% Author: Armando Nicolella, Sergio Savino, LAM4R-University of Napoli
% Federico II
% Last revision: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




function [model]=plane_fit(pts,n)
%%% pts = [Mx3] matrix of the M 3D points to fit the plane
%%% n = optional normal vector to constrain the plane orientation
%%% model = resulting fitted plane model

if exist("n","var")==0
    n=[]; % if normal not provided, set empty
end

options = optimoptions('fminunc','MaxFunctionEvaluations',1e6);

%**************************************************************
% PLANE FITTING
%**************************************************************

if isempty(n)
    % No normal provided: full optimization
    % distance of points to plane
    plan_dist=@(par,points)abs(par(1)*points(:,1)+par(2)*points(:,2)+par(3)*points(:,3)+par(4))/sqrt(par(1)^2+par(2)^2+par(3)^2); 
    % Optimize plane parameters minimizing sum of distances
    plan_par=fminunc(@(x)sum(plan_dist(x,reshape(pts(isnan(pts)==0),[],3))),[1/sqrt(3) 1/sqrt(3) 1/sqrt(3) 0],options); 

else
    % Normal provided: optimize only plane offset

    plan_dist=@(par,points,n)abs(n(1)*points(:,1)+n(2)*points(:,2)+n(3)*points(:,3)+par)/sqrt(n(1)^2+n(2)^2+n(3)^2);
    % Optimize plane offset minimizing sum of distances
    plan_par=fminunc(@(x)sum(plan_dist(x,reshape(pts(isnan(pts)==0),[],3),n)),[0],options); 

end

% Normalize the normal vector
plan_par=plan_par/norm(plan_par(1:3)); 
% Create a planeModel object
model=planeModel(plan_par);
end