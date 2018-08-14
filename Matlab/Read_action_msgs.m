clear all
close all
clc

rosshutdown
rosinit

Action_sub = rossubscriber('/cmd_vel');

disp('Start tracker')

Action_data(1) = receive(Action_sub,100);

tic
for i = 2:1000
    try
        Action_data(i) = receive(Action_sub,10);
        t = toc;
    catch
        break
    end
    disp(i);
end

forward_action_data = [];
rotational_action_data = [];
    
for i = 1:min(length(Action_data))
    forward_action_data = [forward_action_data Action_data(i).Linear.X];
    rotational_action_data = [rotational_action_data Action_data(i).Angular.Z];
end

time = linspace(0,1*length(forward_action_data),length(forward_action_data));

figure
subplot(211)
plot(time,forward_action_data)
subplot(212)
plot(time,rotational_action_data)

subplot(211)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Control action','FontSize',20)
title('Forward action', 'FontSize',20)

subplot(212)
set(gca,'fontsize',15)
xlabel('Time [s]','FontSize',20)
ylabel('Control action','FontSize',20)
title('Angular action', 'FontSize',20)
