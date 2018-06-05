pcl_start_at_0 = pcread('~/Autoware/ros/Maps/start_at_230.pcd');
pcshow(pcl_start_at_0)

% Create a transform object with 45 degrees rotation along z-axis
A = [cos(-0.60) sin(-0.60) 0 -28; ...
     -sin(-0.60) cos(-0.60) 0 514; ...
     0 0 1 0; ...
     0 0 0 1];
% tform = affine3d(A);

% Transform the point cloud
ptCloudOut = pctransform(pcl_start_at_0, A);
    
pcwrite(ptCloudOut, 'start_at_230_transformed.pcd');