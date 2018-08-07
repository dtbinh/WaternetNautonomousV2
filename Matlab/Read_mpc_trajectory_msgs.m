clear all
close all
clc

rosshutdown
rosinit

Ekf_sub = rossubscriber('/mission_coordinator/current_state');

disp('Start tracker')

Ekf_data(1) = receive(Ekf_sub,100);

tic
for i = 2:1000
    try
        Ekf_data(i) = receive(Ekf_sub,10);
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
    position_x_data = [position_x_data Ekf_data(i).X + 30];
    position_y_data = [position_y_data Ekf_data(i).Y - 20];
    angular_vel_data = [angular_vel_data Ekf_data(i).Omega];
    angle_data = [angle_data Ekf_data(i).Theta];
    linear_vel_data = [linear_vel_data Ekf_data(i).U];
end

time = linspace(0,length(Ekf_data),length(Ekf_data));

figure
hold on
subplot(221)
plot(time,position_x_data)
hold on
plot(time,position_y_data)
subplot(222)
plot(time,angular_vel_data)
subplot(223)
plot(time,linear_vel_data)
subplot(224)
plot(time,angle_data)
 
subplot(221)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Position [m]','FontSize',20)
% title('Trajectory of obstacle 1','FontSize',20)
legend('x-coordinate','y-coordinate')

subplot(222)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Angular velocity [rad/s]','FontSize',20)
% title('Trajectory of obstacle 1','FontSize',20)

subplot(223)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Linear velocity [m/s]','FontSize',20)
% title('Trajectory of obstacle 2','FontSize',20)

subplot(224)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Angle [rad]','FontSize',20)
% title('Trajectory of obstacle 2','FontSize',20)