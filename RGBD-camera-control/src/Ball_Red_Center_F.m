function [x,xyz_mean,control] = Ball_Red_Center_F
% =========================================================================
% Function: Ball_Red_Center_F
%
% Description:
%   Acquires RGB-D data from a RealSense camera, computes the aligned point
%   cloud, and extracts the 3D position and control signal of a red ball
%   using vision-based processing (via the find_red function).
%
% Outputs:
%   - x         : 1x2 vector [1, distance (mm)] from zero reference
%   - xyz_mean  : 1x3 vector [x, y, z] of ball center (in meters)
%   - control   : scalar control signal (e.g., deviation from target x)
%
% Dependencies:
%   - find_red.m (ball detection and sphere fitting)
%   - Global RealSense pipeline must be initialized externally
% =========================================================================

% Access global variables (must be initialized elsewhere in the pipeline)

global pipe colorizer pcl_obj align_to alignedFs fs rect

% pipe = realsense.pipeline();
% colorizer = realsense.colorizer();
% pcl_obj = realsense.pointcloud();
%
% profile = pipe.start();
% align_to = realsense.stream.color;
% alignedFs = realsense.align(align_to);
%
% for i = 1:5
% fs = pipe.wait_for_frames();
% end
%
% player1 = pcplayer([-0.5 0.5],[-0.5 0.5],[0 0.5]);
% frameCount =0;

% while frameCount < 2000

%   frameCount = frameCount+1;
% load('rect_realsense.mat')

% === Acquire frames from RealSense camera ===
fs = pipe.wait_for_frames();


% Align depth to color
aligned_frames = alignedFs.process(fs);

% Extract aligned depth frame
depth = aligned_frames.get_depth_frame();

% Extract raw color frame
color = fs.get_color_frame();

% Compute point cloud from depth frame
pnts = pcl_obj.calculate(depth);

% Extract color data and format as Nx3 RGB array
colordata = color.get_data();

colordatavector = [colordata(1:3:end)',colordata(2:3:end)',colordata(3:3:end)'];

% Extract 3D vertices from point cloud
vertices = pnts.get_vertices();

% === Call processing function to detect red ball and compute output ===
[x,xyz_mean,control] = find_red(vertices,colordatavector,depth.get_width(),depth.get_height());
%   xyz_mean(frameCount,:) = out;



end

% pipe.stop();
