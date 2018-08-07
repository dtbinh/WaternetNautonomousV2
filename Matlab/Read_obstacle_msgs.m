clear all
close all
clc

rosshutdown
rosinit

obstacle_detect_sub = rossubscriber('/Obstacle_detection/obstacles');
obstacle_track_sub = rossubscriber('/Obstacle_tracking/obstacles');
obstacle_true_sub = rossubscriber('/Obstacle_detection/true_obstacles');

disp('Start tracker')

tracked_data(1) = receive(obstacle_track_sub,100);
detect_data(1) = obstacle_detect_sub.LatestMessage;
true_data(1) = obstacle_true_sub.LatestMessage;

for i = 2:1000
    try
        tracked_data(i) = receive(obstacle_track_sub,1);
        detect_data(i) = obstacle_detect_sub.LatestMessage;
       true_data(i) = obstacle_true_sub.LatestMessage;
    catch
        break
    end
    disp(i);
end

figure
hold on
for i = 1:min(length(detect_data),length(tracked_data))-1
    subplot(211)
    hold on
    plot([detect_data(i).Obstacles_(1).Pose.Position.X detect_data(i+1).Obstacles_(1).Pose.Position.X], [detect_data(i).Obstacles_(1).Pose.Position.Y detect_data(i+1).Obstacles_(1).Pose.Position.Y],'r-')
    plot([tracked_data(i).Obstacles_(1).Pose.Position.X tracked_data(i+1).Obstacles_(1).Pose.Position.X], [tracked_data(i).Obstacles_(1).Pose.Position.Y tracked_data(i+1).Obstacles_(1).Pose.Position.Y],'b-')

    subplot(212)
    hold on
    plot([detect_data(i).Obstacles_(2).Pose.Position.X detect_data(i+1).Obstacles_(2).Pose.Position.X], [detect_data(i).Obstacles_(2).Pose.Position.Y detect_data(i+1).Obstacles_(2).Pose.Position.Y],'r-')
    plot([tracked_data(i).Obstacles_(2).Pose.Position.X tracked_data(i+1).Obstacles_(2).Pose.Position.X], [tracked_data(i).Obstacles_(2).Pose.Position.Y tracked_data(i+1).Obstacles_(2).Pose.Position.Y],'b-')
    
end

subplot(211)
set(gca,'fontsize',15)
xlabel('X-position [m]','FontSize',20)
ylabel('Y-position [m]','FontSize',20)
title('Trajectory of obstacle 1','FontSize',20)
legend('Detected obstacle','Tracked obstacle')
axis equal

subplot(212)
set(gca,'fontsize',15)
xlabel('X-position [m]','FontSize',20)
ylabel('Y-position [m]','FontSize',20)
title('Trajectory of obstacle 2','FontSize',20)
legend('Detected obstacle','Tracked obstacle')
axis equal

figure
for i = 1:min(length(detect_data),length(tracked_data))-1
    subplot(211)
    hold on
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(1).Pose.Orientation.Z detect_data(i+1).Obstacles_(1).Pose.Orientation.Z],'r-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(1).Pose.Orientation.Z tracked_data(i+1).Obstacles_(1).Pose.Orientation.Z],'g-')
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(2).Pose.Orientation.Z detect_data(i+1).Obstacles_(2).Pose.Orientation.Z],'k-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(2).Pose.Orientation.Z tracked_data(i+1).Obstacles_(2).Pose.Orientation.Z],'b-')
    %plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(3).Pose.Orientation.Z tracked_data(i+1).Obstacles_(3).Pose.Orientation.Z],'g-')

    subplot(212)
    hold on
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(1).Twist.Linear.X detect_data(i+1).Obstacles_(1).Twist.Linear.X],'r-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(1).Twist.Linear.X tracked_data(i+1).Obstacles_(1).Twist.Linear.X],'g-')
    plot([i * 0.2 (i+1) * 0.2], [detect_data(i).Obstacles_(2).Twist.Linear.X detect_data(i+1).Obstacles_(2).Twist.Linear.X],'k-')
    plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(2).Twist.Linear.X tracked_data(i+1).Obstacles_(2).Twist.Linear.X],'b-')
    %plot([i * 0.2 (i+1) * 0.2], [tracked_data(i).Obstacles_(3).Twist.Linear.X tracked_data(i+1).Obstacles_(3).Twist.Linear.X],'g-')

end

subplot(211)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Angle [rad]','FontSize',20)
title('True and tracked heading','FontSize',20)
legend('Detected obstacle 1','Tracked obstacle 1','Detected obstacle 2','Tracked obstacle 2')
axis([0 (i+1) * 0.2 -1 pi+1])

subplot(212)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Velocity [m/s]','FontSize',20)
title('True and tracked velocity','FontSize',20)
legend('Detected obstacle 1','Tracked obstacle 1','Detected obstacle 2','Tracked obstacle 2') 
axis([0 (i+1) * 0.2 0 2])

pos_error_max = 0;
pos_error_avg = 0;
for i = 1:min(length(tracked_data),length(true_data))-1
    error = sqrt((true_data(i).Obstacles_(1).Pose.Position.X - tracked_data(i).Obstacles_(1).Pose.Position.X)^2 + (true_data(i).Obstacles_(1).Pose.Position.Y - tracked_data(i).Obstacles_(1).Pose.Position.Y)^2);
    if (error > pos_error_max)
        pos_error_max = error;
    end
    pos_error_avg = pos_error_avg + error;
end

text=sprintf('Maximum position error: %0.3d', pos_error_max);
disp(text)
text=sprintf('Average position error: %0.3d', pos_error_avg / (min(length(detect_data),length(tracked_data))-1));
disp(text)