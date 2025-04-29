%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% captureCalibrationPoseClouds.m
%
% Hand-eye-calibration data grabber for:
%   – Dobot Magician (DLL interface)
%   – Intel RealSense D415 RGB-D camera
%
% For each of nine predefined robot poses the script:
%   1. Connects to the Dobot, queues a PTP move, waits for convergence
%   2. Queries the actual tool pose (after offsets) from joint encoders
%   3. Flushes 30 warm-up frames, then records 10 RGB-D frames
%   4. Projects depth into XYZ, packages coloured pointCloud objects
%   5. Stores:
%        • I{pose,frame}  → 10 point clouds × 9 poses
%        • Ppose(:,pose) → tool-centre-points in robot frame
%
% All subsequent scripts (averaging, segmentation, plane fit, ICP, AX=XB)
% assume these two .mat files.
%
% Author: Armando Nicolella, Chiara Cosenza LAM4R-University of Napoli Federico II • Last rev: 28 Apr 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
close all           
clear               
clc                 
                    
%% RealSense configuration
cfg = realsense.config();                               % create empty config
% alternative resolutions (commented out for quick switch)
% cfg.enable_stream(realsense.stream.color,-1,1280,720,realsense.format.rgb8,30);
% cfg.enable_stream(realsense.stream.depth,-1,1280,720,realsense.format.z16,30);
cfg.enable_stream(realsense.stream.color,-1,640,480,realsense.format.rgb8,30);
cfg.enable_stream(realsense.stream.depth,-1,640,480,realsense.format.z16,30);
% cfg.enable_stream(realsense.stream.color,-1,424,240,realsense.format.rgb8,30);
% cfg.enable_stream(realsense.stream.depth,-1,424,240,realsense.format.z16,30);

pipe    = realsense.pipeline();                         % pipeline
profile = pipe.start(cfg);                              % start streaming
pause(5)                                                % let exposure settle

align_to  = realsense.stream.color;                     % depth 2 colour frame
alignedFs = realsense.align(align_to);

%% Target poses (mm) for calibration
P = [265  0   10;
     275  100 20;
     275 -100 30;
     175 -100 0;
     175  100 30;
     225  100 20;
     275  0   40;
     225 -100 0;
     175  0   10];

Ppose = ones(4,size(P,1));                              % pre-allocate pose matrix

%% Loop over each pose
for i = 1:size(P,1)
    pause(3)                                            % short delay
    
    %–––––––– Connect and move Dobot ––––––––%
    if ~libisloaded('DobotDll')
        loadlibrary('DobotDll.dll','DobotDll.h');       % load DLL if needed
    end
    
    ch   = blanks(128);                                 % temp char buffer
    str1 = libpointer('cstring',ch);
    [~,res2] = calllib('DobotDll','SearchDobot',str1,128);  % find robot
    str2 = libpointer('cstring',res2);
    calllib('DobotDll','ConnectDobot',str2,115200);         % open serial
    
    queue_index = 0;                                    % init queue
    
    ptp.ptpMode = 1;                                    % jump mode move
    ptp.x = P(i,1); 
    ptp.y = P(i,2); 
    ptp.z = P(i,3); 
    ptp.r = 90;
    
    calllib('DobotDll','SetQueuedCmdStartExec');        % start queue
    ptpstruct    = libstruct('tagPTPCmd',ptp);
    ptpstructptr = libpointer('tagPTPCmdPtr',ptpstruct);
    queue_ptr    = libpointer('uint64Ptr',queue_index);
    calllib('DobotDll','SetPTPCmd',ptpstructptr,true,queue_ptr);
    pause(2)                                            % wait for motion
    
    posestruct.x = 1; 
    posestruct.y = 1; 
    posestruct.z = 1;
    posestruct.jointAngle = [1;1;1;1];
    posestruct = libstruct('tagPose',posestruct);
    calllib('DobotDll','GetPose',posestruct);           % read actual pose
    calllib('DobotDll','SetQueuedCmdStopExec');         % stop queue
    
    posex = posestruct.x; 
    posey = posestruct.y; 
    posez = posestruct.z;
    
    Ppose(1,i) = posex + 8.85;                          % apply marker offsets
    Ppose(2,i) = posey;
    Ppose(3,i) = posez + 45;
    
    calllib('DobotDll','DisconnectDobot');              % release robot
    
    clear ptpstruct ptpstructptr str1 str2 queue_ptr    % free pointers
    pause(3)                                            % let vibrations stop
    
    %–––––––– Capture 10 RGB-D frames for each position ––––––––%
    for t = 1:30
        pipe.wait_for_frames();                         % flush warm-up frames
    end
    
    for f = 1:10
        pointcloud = realsense.pointcloud();            % create pc object
        fs = pipe.wait_for_frames();                    % grab synced frames
        
        aligned_frames = alignedFs.process(fs);         % align depth to colour
        depth  = aligned_frames.get_depth_frame();
        color  = fs.get_color_frame();
        
        if depth.logical() && color.logical()  % check that both depth and color frames are valid
            img = permute(reshape((color.get_data())',...
                   [3,color.get_width(),color.get_height()]),[3 2 1]);  % reshape color data into a 3D RGB image [height x width x 3]
            pointcloud.map_to(color); % associate color texture with the point cloud
            points   = pointcloud.calculate(depth); % compute 3D point cloud from depth frame
            vertices = points.get_vertices();  % extract XYZ coordinates of all points
        end
        
        if ~isempty(vertices)    % proceed only if valid 3D points were generated
            X = vertices(:,1,1);  % extract coordinates
            Y = vertices(:,2,1); 
            Z = vertices(:,3,1);
            XYZ(:,:,1) = (reshape(X',depth.get_width(),depth.get_height()))';  % reshape coordinates into image format
            XYZ(:,:,2) = (reshape(Y',depth.get_width(),depth.get_height()))';
            XYZ(:,:,3) = (reshape(Z',depth.get_width(),depth.get_height()))';
            XYZ_cloud  = pointCloud(XYZ,'Color',img);   % build MATLAB pc object
        end
        
        I{i,f} = XYZ_cloud;                             % store cloud
    end
    
    clear pointcloud fs depth points vertices XYZ       % free memory
end

%% Shutdown camera and save results
pipe.stop                                              % stop RealSense pipeline
save('10pc_9positions','I');                           % point clouds
save('Ppose','Ppose');                                 % robot poses