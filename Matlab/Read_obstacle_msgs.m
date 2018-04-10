clear all
close all
clc

rosshutdown
rosinit

obstacle_detect_sub = rossubscriber('/Obstacle_detection/obstacles');
obstacle_track_sub = rossubscriber('/Obstacle_tracking/obstacles');

disp('Start tracker')

tracked_data(1) = receive(obstacle_track_sub,100);
detect_data(1) = obstacle_detect_sub.LatestMessage;

for i = 2:1000
    try
        tracked_data(i) = receive(obstacle_track_sub,1);
        detect_data(i) = obstacle_detect_sub.LatestMessage;
    catch
        break
    end
    disp(i);
end

figure
hold on
for i = 1:min(length(detect_data),length(tracked_data))-1
    subplot(221)
    hold on
    plot([detect_data(i).Obstacles_(1).Pose.Position.X detect_data(i+1).Obstacles_(1).Pose.Position.X], [detect_data(i).Obstacles_(1).Pose.Position.Y detect_data(i+1).Obstacles_(1).Pose.Position.Y],'r-')
    plot([tracked_data(i).Obstacles_(1).Pose.Position.X tracked_data(i+1).Obstacles_(1).Pose.Position.X], [tracked_data(i).Obstacles_(1).Pose.Position.Y tracked_data(i+1).Obstacles_(1).Pose.Position.Y],'b-')

    subplot(222)
    hold on
    plot([detect_data(i).Obstacles_(2).Pose.Position.X detect_data(i+1).Obstacles_(2).Pose.Position.X], [detect_data(i).Obstacles_(2).Pose.Position.Y detect_data(i+1).Obstacles_(2).Pose.Position.Y],'r-')
    plot([tracked_data(i).Obstacles_(2).Pose.Position.X tracked_data(i+1).Obstacles_(2).Pose.Position.X], [tracked_data(i).Obstacles_(2).Pose.Position.Y tracked_data(i+1).Obstacles_(2).Pose.Position.Y],'b-')
    
    subplot(223)
    hold on
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(1).Pose.Orientation.Z detect_data(i+1).Obstacles_(1).Pose.Orientation.Z],'r-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(1).Pose.Orientation.Z tracked_data(i+1).Obstacles_(1).Pose.Orientation.Z],'b-')
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(2).Pose.Orientation.Z detect_data(i+1).Obstacles_(2).Pose.Orientation.Z],'k-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(2).Pose.Orientation.Z tracked_data(i+1).Obstacles_(2).Pose.Orientation.Z],'g-')
    
    subplot(224)
    hold on
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(1).Twist.Linear.X detect_data(i+1).Obstacles_(1).Twist.Linear.X],'r-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(1).Twist.Linear.X tracked_data(i+1).Obstacles_(1).Twist.Linear.X],'b-')
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(2).Twist.Linear.X detect_data(i+1).Obstacles_(2).Twist.Linear.X],'k-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(2).Twist.Linear.X tracked_data(i+1).Obstacles_(2).Twist.Linear.X],'g-')
end
subplot(221)
xlabel('X-position [m]')
ylabel('Y-position [m]')
legend('Detected obstacle','Tracked obstacle')
axis equal

subplot(222)
xlabel('X-position [m]')
ylabel('Y-position [m]')
legend('Detected obstacle','Tracked obstacle')
axis equal

subplot(223)
xlabel('Time [s]')
ylabel('Angle [rad]')
legend('Detected obstacle 1','Tracked obstacle 1','Detected obstacle 2','Tracked obstacle 2')

subplot(224)
xlabel('Time [s]')
ylabel('Velocity [m/s]')
legend('Detected obstacle 1','Tracked obstacle 1','Detected obstacle 2','Tracked obstacle 2') 