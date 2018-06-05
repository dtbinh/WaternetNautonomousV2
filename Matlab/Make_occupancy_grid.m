clear all
close all
clc

rosshutdown
rosinit

obstacle_detect_sub = rossubscriber('/ndt_map');
pcl_msg = receive(obstacle_detect_sub,10);
xyz = readXYZ(pcl_msg);

disp("data received");

Xmax = max(xyz(:,1)) + 0.01;
Xmin = min(xyz(:,1)) - 0.01;
Ymax = max(xyz(:,2)) + 0.01;
Ymin = min(xyz(:,2)) - 0.01;

for i = 1:length(xyz)
    if ((xyz(i,3) < 3) && (xyz(i,3) > -10))
        grid(ceil(xyz(i,1) - Xmin),ceil(xyz(i,2) - Ymin)) = 1;
    end
end
    
figure
subplot(121)
scatter3(pcl_msg);
subplot(122)
surf(grid)
view(2)

