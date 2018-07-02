clear all
close all
clc

rosshutdown
rosinit

Ekf_sub = rossubscriber('/Ekf/next_state');
actuator_sub = rossubscriber('/PID/next_state');

disp('Start tracker')

Ekf_data(1) = receive(Ekf_sub,100);
actuator_data(1) = actuator_sub.LatestMessage;

tic
for i = 2:1000
    try
        Ekf_data(i) = receive(Ekf_sub,1);
        actuator_data(i) = actuator_sub.LatestMessage;
        t = toc;
    catch
        break
    end
    disp(i);
end

position_x_data = [];
position_y_data = [];
angular_vel_data = [];
angle_data = [];
linear_vel_data = [];
left_actuator_data = [];
right_actuator_data = [];
    
for i = 1:min(length(Ekf_data))
    position_x_data = [position_x_data Ekf_data(i).X];
    position_y_data = [position_y_data Ekf_data(i).Y];
    angular_vel_data = [angular_vel_data Ekf_data(i).Omega];
    angle_data = [angle_data Ekf_data(i).Theta];
    linear_vel_data = [linear_vel_data Ekf_data(i).U];
end

for i = 1:min(length(actuator_data))
    left_actuator_data = [left_actuator_data actuator_data(i).TL];
    right_actuator_data = [right_actuator_data actuator_data(i).TR];
end
time = linspace(0,0.1*length(Ekf_data),length(Ekf_data));

figure
hold on
subplot(321)
plot(time,position_x_data)
hold on
plot(time,position_y_data)
subplot(322)
plot(time,angular_vel_data)
subplot(323)
plot(time,linear_vel_data)
subplot(324)
plot(time,angle_data)
subplot(325)
plot(time,left_actuator_data)
subplot(326)
plot(time,right_actuator_data)
 
subplot(321)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Position [m]','FontSize',20)
% title('Trajectory of obstacle 1','FontSize',20)
legend('x-coordinate','y-coordinate')

subplot(322)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Angular velocity [rad/s]','FontSize',20)
% title('Trajectory of obstacle 1','FontSize',20)

subplot(323)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Linear velocity [m/s]','FontSize',20)
% title('Trajectory of obstacle 2','FontSize',20)

subplot(324)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Angle [rad]','FontSize',20)
% title('Trajectory of obstacle 2','FontSize',20)

subplot(325)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Left actuator','FontSize',20)
% title('Trajectory of obstacle 2','FontSize',20)

subplot(326)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Right actuator','FontSize',20)
% title('Trajectory of obstacle 2','FontSize',20)