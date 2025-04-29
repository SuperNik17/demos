%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% extractMarkerFromRGB.m
%
% This script performs color-based segmentation to isolate a red planar
% marker from a set of averaged RGB-D point clouds. 
% It applies local contrast enhancement, color channel subtraction, 
% sharpening, binarization, and morphological filtering.
%
% Outputs:
%   - averagedPC.mat: cell array containing the extracted marker point clouds.
%
% Author: Armando Nicolella, Chiara Cosenza, LAM4R-University of Napoli
% Federico II
% Last revision: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all         
clear             
clc               

%% Load averaged point clouds
load('averagedPC.mat');


%% Color segmentation and point cloud extraction

for n = 1:length(averagedPC)    % loop over each averaged point cloud

    % Enhance local contrast (helps marker separation)
    B = localcontrast(pc_mediato{1,n}.Color); % improve local contrast of RGB image
    
    % Extract red channel
    redChannel = B(:,:,1);      % isolate red component

    % Convert to grayscale
    grayFrame = rgb2gray(B);    % convert RGB image to grayscale

    % Subtract grayscale from red channel
    diffFrame = imsubtract(redChannel, grayFrame); % highlight red areas
    
    % Sharpen the difference image
    diffFrameSharp = imsharpen(diffFrame); % enhance edges and features
                                           % % tuning might be necessary for aggressive sharpening
                                           
    % Binarization
    binFrame = imbinarize(diffFrameSharp); % convert to binary image based on intensity threshold
                                           % % tuning might be necessary depending on lighting
    
    % Remove small objects
    BW1 = bwareaopen(binFrame, 50); % eliminate small noise regions
                                    % % 50 pixel threshold: tuning might be necessary
    
    % Morphological convex hull (close marker borders)
    BW2 = bwconvhull(BW1);          % fill gaps around the segmented marker
    
    figure
    imshow(BW2)                     % display binary mask (optional)

    % Extract marker point cloud
    [col, row] = find(BW2 == 1);     % find pixel coordinates of segmented region
    Points = [col, row];             % pixel list (row = y, col = x)
    
    clear xCoord yCoord zCoord
    % For each pixel belonging to the segmented marker, extract the corresponding
    % 3D coordinates (X, Y, Z) from the averaged point cloud.
    % Pixel coordinates are mapped to camera frames using the Location matrix.
    for j = 1:length(Points)
        xCoord(j,1) = pc_mediato{1,n}.Location(floor(Points(j,1)), floor(Points(j,2)), 1);    
        yCoord(j,1) = pc_mediato{1,n}.Location(floor(Points(j,1)), floor(Points(j,2)), 2);    
        zCoord(j,1) = pc_mediato{1,n}.Location(floor(Points(j,1)), floor(Points(j,2)), 3);     
    end

    % Combine extracted XYZ coordinates
    RED = [xCoord, yCoord, zCoord];

    % Create point cloud object
    marker = pointCloud(RED);        % build point cloud from marker points
    extractedMarkers{1,n} = marker;  % store in output cell array

end

%% Save results
save('averaged_pcmarker', 'extractedMarkers'); % save extracted marker point clouds
