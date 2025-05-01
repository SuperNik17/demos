function [x,xyz_mean,control] = find_red(vertices,colordatavector,width,height,rect)
% =========================================================================
% Function: find_red
%
% Description:
%   Processes the point cloud and RGB data to detect a red object (ball),
%   extract its 3D location, and compute control signals. Works by cropping
%   the region of interest, segmenting based on red channel dominance, and
%   fitting a sphere to the extracted 3D points.
%
% Inputs:
%   - vertices        : Nx3 array of 3D point cloud coordinates
%   - colordatavector : Nx3 array of corresponding RGB values
%   - width, height   : resolution of the depth frame
%   - rect            : ROI crop rectangle (manually set)
%
% Outputs:
%   - x         : [1, distance_mm] where distance is from a reference
%   - xyz_mean  : 3D center of the ball
%   - control   : feedback signal (x-difference from reference)
% =========================================================================

% Reshape flat arrays into proper 2D image format (height x width)
XYZ(:,:,1)=(reshape(vertices(:,1),width,height))';
XYZ(:,:,2)=(reshape(vertices(:,2),width,height))';
XYZ(:,:,3)=(reshape(vertices(:,3),width,height))';

IMG(:,:,1)=(reshape(colordatavector(:,1),width,height))';
IMG(:,:,2)=(reshape(colordatavector(:,2),width,height))';
IMG(:,:,3)=(reshape(colordatavector(:,3),width,height))';
%XYZ_cloud = pointCloud(XYZ,'Color',IMG);

% Load cropping rectangle (ROI) to isolate ball region
load('rect_realsense.mat')

% Crop image and point cloud to region of interest
colorCrop = imcrop(IMG,rect);
locationCrop = imcrop(XYZ,rect);

% Create point cloud from cropped region
%Punti = locationCrop;
pointcloud=pointCloud(locationCrop,'Color',colorCrop);


% Segment red channel: subtract red from grayscale to isolate red dominance
I = rgb2gray(colorCrop(:,:,:) - colorCrop(:,:,1));
%     [centers,radii,metric] = imfindcircles(I==0,[10 length(I)]);
%
%     centersC{i}=centers;

[col,row] = find(I==0);
Points = [col row];


% Extract 3D points corresponding to red pixels
for j =1: length(Points)
    
    x_ball(j,1) = pointcloud.Location(Points(j,1),Points(j,2), 1);
    y_ball(j,1) = pointcloud.Location(Points(j,1), Points(j,2), 2);
    z_ball(j,1) = pointcloud.Location(Points(j,1), Points(j,2) ,3);
    
end

% Assemble ball points and create a point cloud
Ball =[x_ball y_ball z_ball];
Ball_Cloud = pointCloud(Ball);
% Filter the cloud with an ROI (e.g., height constraint)
roi = [-inf inf -inf inf 0.35 0.4]; %ATTENZIONE: Il roi va regolato
indices = findPointsInROI(Ball_Cloud,roi);
Ball_Cloud_clean = select(Ball_Cloud,indices);
%     save(filename{i},'Ball_Cloud_clean')

% Fit a sphere to the filtered cloud to estimate ball center
[x1,y1,z1] = sphereFit_1(Ball_Cloud_clean.Location,0.020);

% Set the mean ball position
xyz_mean = [x1 y1 z1];


% Load reference positions
load('posizione_mid_red_ball.mat')  % Mid stroke position
load('posizione_zero_red_ball.mat') % Zero reference

% Compute 3D distance from zero position (used as feedback)
x =  pdist2(xyz_mean,xyz_mean_zero)*1000;
% x =  pdist2(xyz_mean,xyz_mean_mid)*1000;
% Also compute control signal as difference from mid
control = x1 - xyz_mean_mid(1);
% Pack output
x = [1 x];


end

% control = abs((norm(Q - P1) * 10^3))
% 
% % %% Sort of P
% 
% if control <= x_cm/4
%     
%     x_cm = x_cm;
%     
% elseif (control > x_cm/4) && (control <= x_cm/2)
%     
%     x_cm = abs(x_cm - x_cm/6);
%     
% else
%     
%     x_cm = abs(x_cm - x_cm/3);
%     
% end









% % %De commenta due volte
% % diff_red = imsubtract(img(:,:,1), rgb2gray(img)); %diff_im
% % % diff_redfilt = medfilt2(diff_red); %diff_im1
% % diff_redbinarize = imbinarize(diff_red,0.10);
% % % diff_redbinarize = imbinarize(diff_redfilt,0.10); %diff_im2
% % % diff_redbwareopen = bwareaopen(diff_redbinarize,30); %diff_im3
% % % bw_red = bwlabel(diff_redbwareopen,8);
% % 
% % PuntiRossi = vertices;
% % % PuntiRossi(repmat(~diff_redbwareopen,1,3))=NaN;
% % PuntiRossi(repmat(~diff_red,1,3))=NaN;
% % 
% % LCwPL_red=PuntiRossi;
% % % pointcloudRed = pointCloud(reshape(LCwPL_red,size(img,1),size(img,2),3));
% % pointcloudRed = pointCloud(LCwPL_red);
% % % pointcloudRed.Color = img;
% % % pcshow(pointcloudRed,'MarkerSize',70)
% % 
% % % roiz = [-inf inf -inf inf -inf inf];
% % % sampleIndicesRed = findPointsInROI(pointcloudRed,roiz);
% % % ptcloudZcutRed = select(pointcloudRed,sampleIndicesRed);
% % 
% % r = 0.0020; %Ball Radius = 20 mm
% % 
% % 
% % % x = ptcloudZcutRed.Location(:,1);
% % % y = ptcloudZcutRed.Location(:,2);
% % % z = ptcloudZcutRed.Location(:,3);
% % 
% % 
% % 
% % % points=[x,y,z];
% % 
% % % cylCloud = pointCloud(points);
% % normals = pcnormals(pointcloudRed,10);
% % 
% % % Internal Normals 
% % % center=[mean(mean(x)) mean(mean(y)) mean(mean(z))];
% % % p1 = center - points;
% % center=[mean(LCwPL_red(:,1)),mean(LCwPL_red(:,2)),mean(LCwPL_red(:,3))];
% % 
% % p1 = center -LCwPL_red;
% % % p1=reshape(p1,size(img,1),size(img,2),3);
% % % normals2=reshape(normals,size(img,1)*size(img,2),3);
% % 
% % pp=cross(p1',normals');
% % 
% % % pp=vecnorm(pp)';
% % pp=(sqrt(pp(1,:).^2+pp(2,:).^2+pp(3,:).^2))';
% % 
% % angle=atan2(pp,sum(p1.*normals,2));
% % % angle=atan2(vecnorm(cross(p1',normals'))',sum(p1.*normals,2));
% % % angle=atan2(vecnorm(cross(p1',normals2'))',sum(p1.*normals2,2));
% % % angle=atan2(vecnorm(cross(p1,normals)),sum(p1.*normals,2));
% % 
% % condition=repmat(-2*(angle > pi/2 | angle < -pi/2)+1,1,3);
% % normals=normals.*condition;   
% % 
% % displacements=normals*r;
% % 
% % xpr=LCwPL_red(:,1)+displacements(:,1);
% % ypr=LCwPL_red(:,2)+displacements(:,2);
% % zpr=LCwPL_red(:,3)+displacements(:,3);
% % xyz_mean = mean([xpr ypr zpr]);
