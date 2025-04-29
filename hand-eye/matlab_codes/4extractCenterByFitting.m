%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% extractCenterByFitting.m
%
% This script aligns the segmented experimental marker clouds with a 
% artificial marker model using plane projection and ICP registration.
% It computes the marker centers either:
%   - geometrically (via ICP transformation applied to the origin)
%   - statistically (as mean of experimental points).
%
% Outputs:
%   - centro_fit.mat: center coordinates estimated via ICP fitting
%
% Author: Armando Nicolella, Chiara Cosenza, Segio Savino,
% LAM4R-University of Naples Federico II
% Last revision: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

%%
filename = 'test4';          % folder where outputs will be saved

% load([filename,'pcmarker_mediato']) 
load('averaged_pcmarker.mat')   % load extracted marker point clouds

%% Create artificial marker point cloud (B)
l = 11;    % mm, shorter side of marker rectangle
h = 32;    % mm, longer side of marker rectangle
nx = 40;   % number of points along x (grid resolution)  % can be changed
ny = 80;   % number of points along y (grid resolution)  % can be changed
r = 1.5;   % mm, fillet radius for rounded corners

%% Artificial Marker building
% compute points of the rounded corner arc (X and Y) for 0° to 90°
xr = cosd(0:90) * r;
yr = sind(0:90) * r;

xv = [(l/2-r+xr)'; (-l/2+r-fliplr(xr))'; (-l/2+r-xr)'; (l/2-r+fliplr(xr))']; % build x coordinates of the corners
yv = [(h/2-r+yr)'; (h/2-r+fliplr(yr))'; (-h/2+r-yr)'; (-h/2+r-fliplr(yr))']; % build y coordinates of the corners
xv(end+1) = xv(1);
yv(end+1) = yv(1);

% numeber of point for each dimension
x = linspace(-l/2, l/2, nx);
y = linspace(-h/2, h/2, ny);


%mesh grid to uniform the artificial marker
[X, Y] = meshgrid(x, y);
Z = zeros(size(X));

% Create mask of marker shape 
mask = inpolygon(X, Y, xv, yv);
X(mask < 1) = NaN;
Y(mask < 1) = NaN;
Z(mask < 1) = NaN;

% Flatten the 2D arrays and remove NaNs
X = reshape(X,[],1);
Y = reshape(Y,[],1);
Z = reshape(Z,[],1);
X = X(~isnan(X));
Y = Y(~isnan(Y));
Z = Z(~isnan(Z));

% Create artificial point cloud
B = pointCloud([X/1000 Y/1000 Z/1000]); % converted to meters

%% Denoise experimental clouds (A)
for j = 1:length(G)
    G1 = pcdenoise(G{1,j}, "NumNeighbors", 12); % remove outliers % tuning might be necessary
    G{j,1} = G1;
end

%% Estimate common plane across all markers
points = [];
for k = 1:length(G)
    points = [points; G{k}.Location - mean(G{k}.Location)]; % centralize the cloud
end
model = plane_fit(points); % fit best plane to all points

%% Register each experimental cloud (G) with the artificial model (B)
for k = 1:length(G)
    clear tform
    A = G{k};

    % Centralize experimental cloud
    trasl = mean(A.Location); % experimental centroid
    A = pointCloud(A.Location - mean(A.Location));

    % Project experimental cloud onto estimated common plane
    zp = -(model.Parameters(1)*A.Location(:,1) + model.Parameters(2)*A.Location(:,2) + model.Parameters(4)) / model.Parameters(3); % compute the Z-coordinate for each point so that it lies exactly on the estimated plane
    Ap = pointCloud([A.Location(:,1), A.Location(:,2), zp]);

    % ICP registration: theoretical → projected experimental
    tform = pcregistericp(B, Ap, MaxIterations=500, Tolerance=[0.001 0.01]); % tuning might be necessary
    
    % Apply transformation and reinsert original translation
    B_moved = pctransform(B, tform);
    B_moved = pointCloud(B_moved.Location + trasl);

    % Estimate center via ICP transformation
    center(:,k) = tform.T * [0;0;0;1] + [trasl';0];

    % Estimate center via simple mean of points
    center_mean(:,k) = [mean(G{k}.Location)'; 1];

    % Save results for ICP-based center
    centro_fit(1,k) = center(1,k) * 1000; % mm
    centro_fit(2,k) = center(2,k) * 1000;
    centro_fit(3,k) = center(3,k) * 1000;
    centro_fit(4,k) = center(4,k);

    % Save results for mean-based center
    centro_media(1,k) = center_mean(1,k) * 1000; % mm
    centro_media(2,k) = center_mean(2,k) * 1000;
    centro_media(3,k) = center_mean(3,k) * 1000;
    centro_media(4,k) = center_mean(4,k);

    % Plot overlay of experimental and theoretical cloud
    figure
    pcshowpair(G{k}, B_moved, 'MarkerSize', 15)
    hold on
    plot3(center(1,k), center(2,k), center(3,k), 'or') % plot estimated center
    grid on
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold off
    print(num2str(k),'-dpng') % save plot as PNG
end

%% Save results
save('centro_fit', 'centro_fit');
% save([filename,'centro_fit'],'centro_fit');
movefile('*.png', filename)    % move PNGs to output folder
movefile('*.mat', filename)    % move MAT files to output folder