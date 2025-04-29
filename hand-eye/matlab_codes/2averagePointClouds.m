%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% averagePointClouds.m
%
% This script loads multiple RGB-D point clouds acquired for each robot pose,
% averages them along the frame dimension to reduce random noise, and creates
% a single, denoised point cloud per pose.
%
% Outputs:
%   - averagedPointClouds.mat: cell array containing one averaged point cloud for each pose.
%
% Author: Armando Nicolella, Chiara Cosenza, Sergio Savino,
% LAM4R-University of Napoli Federico II
% Last revision: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all          
clear              
clc                

%% Load point clouds
load("10pc_9positions.mat") % load the captured point clouds

%% Extraction of X, Y, Z matrices
for i = 1:size(I,1)            % loop over each pose
    for j = 1:size(I,2)        % loop over the 10 frames for each pose
        x = (I{i,j}.Location(:,:,1)); % extract X coordinates
        y = (I{i,j}.Location(:,:,2)); 
        z = (I{i,j}.Location(:,:,3)); 

        X_all(:,:,j) = x;      % store matrices X along third dimension
        Y_all(:,:,j) = y;      
        Z_all(:,:,j) = z;      
    end 
    Position{i,1} = X_all;     % save all X matrices for pose i
    Position{i,2} = Y_all;     
    Position{i,3} = Z_all;     
end

%% Compute mean along the third dimension
for k = 1:size(Position,1)
    meanX = mean(Position{k,1},3,"omitnan"); % compute mean of X matrices
    meanY = mean(Position{k,2},3,"omitnan"); 
    meanZ = mean(Position{k,3},3,"omitnan"); 

    MeanMatrices{k,1} = meanX; % save mean X matrix
    MeanMatrices{k,2} = meanY; 
    MeanMatrices{k,3} = meanZ; 
end 

%% Creation of averaged point clouds
for m = 1:size(MeanMatrices,1)
    meanX = MeanMatrices{m,1};  % extract mean X matrix
    meanY = MeanMatrices{m,2};  % extract mean Y matrix
    meanZ = MeanMatrices{m,3};  % extract mean Z matrix
    XYZ{1,m} = cat(3,meanX,meanY,meanZ); % concatenate X, Y, Z into a 3D matrix
    averagedPointClouds{1,m} = pointCloud(XYZ{1,m},'Color',I{m,1}.Color); % create point cloud object with color
end

%% Save result
save('averagedPC','averagedPointClouds'); % save the averaged point clouds
