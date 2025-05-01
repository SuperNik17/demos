function [x1,y1,z1] = sphereFit_1(Ball_Cloud_clean,r);
% =========================================================================
% Function: sphereFit_1
%
% Description:
%   Estimates the center of a spherical object given a set of 3D points
%   sampled from its surface. The method computes normals to the surface,
%   orients them inward, and projects points along these normals by a
%   known radius 'r'. The average of the projected points gives the center.
%
% Inputs:
%   - Ball_Cloud_clean : pointCloud object (filtered red ball surface)
%   - r                : radius of the sphere (in meters, e.g., 0.020)
%
% Outputs:
%   - x1, y1, z1 : coordinates of the estimated center of the sphere
%
% =========================================================================

% Extract XYZ coordinates from the point cloud

x = Ball_Cloud_clean.Location(:,1);
y = Ball_Cloud_clean.Location(:,2);
z = Ball_Cloud_clean.Location(:,3);


% Combine into a Nx3 array of 3D points
points=[x,y,z];

% Recreate pointCloud object to compute normals
cylCloud = pointCloud(points);
% Estimate local surface normals (10 nearest neighbors)
normals = pcnormals(cylCloud,10);

% Compute centroid of the input cloud (approximate sphere center)
center=[mean(mean(x)) mean(mean(y)) mean(mean(z))];

% Vector from each point to the estimated center
p1 = center - points;

% Compute angle between normals and vectors to center
angle=atan2(vecnorm(cross(p1',normals'))',sum(p1.*normals,2));
% Flip normals that are pointing outward
condition=repmat(-2*(angle > pi/2 | angle < -pi/2)+1,1,3);
normals=normals.*condition;
% Project each point inward along its normal by radius 'r'
displacements=normals*r;

% New (projected) points ideally located at the center of the sphere
xpr=x+displacements(:,1);
ypr=y+displacements(:,2);
zpr=z+displacements(:,3);

% Compute mean of all projected points = estimated center
xyz_mean(k,:) = mean([xpr ypr zpr]);

% Output estimated center coordinates
x1 = xyz_mean(1);
y1 = xyz_mean(2);
z1 = xyz_mean(3);

end