% clear all
% close all
% clc
% 
% rosshutdown
% rosinit
% 
% kkt_sub = rossubscriber('/MPC/kkt');
% time_sub = rossubscriber('/MPC/solvertime');
% 
% disp('Start tracker')
% 
% kkt_data(1) = receive(kkt_sub,100);
% time_data(1) = time_sub.LatestMessage;
% 
% for i = 2:1000
%     try
%         kkt_data(i) = receive(kkt_sub,10);
%         time_data(i) = time_sub.LatestMessage;
%     catch
%         break
%     end
%     disp(i);
% end
% 
% for i = 1:length(kkt_data)
%     kkt_mpc_data(i) = kkt_data(i).Data;
% end
% 
% 
% for i = 1:length(time_data)
%     time_mpc_data(i) = time_data(i).Data;
% end
% 
% save('mpc25.mat');

figure
load('straight_line_through_obstacle/mpcc25_with_hotstart.mat')
semilogy(linspace(0,length(kkt_mpc_data),length(kkt_mpc_data)),kkt_mpc_data,'r')

load('straight_line_through_obstacle/mpc25_with_hotstart.mat')
hold on
semilogy(linspace(0,length(kkt_mpc_data),length(kkt_mpc_data)),kkt_mpc_data, 'b')
% 
% load('straight_line_through_obstacle/mpc25_no_hotstart.mat')
% semilogy(linspace(0,length(kkt_mpc_data),length(kkt_mpc_data)),kkt_mpc_data,'g')

load('straight_line_through_obstacle/mpc50_with_hotstart.mat')
semilogy(linspace(0,length(kkt_mpc_data),length(kkt_mpc_data)),kkt_mpc_data,'k')

load('straight_line_through_obstacle/mpc25_small_weight.mat')
semilogy(linspace(0,length(kkt_mpc_data),length(kkt_mpc_data)),kkt_mpc_data, 'm')

load('straight_line_through_obstacle/mpc50_small_weight.mat')
semilogy(linspace(0,length(kkt_mpc_data),length(kkt_mpc_data)),kkt_mpc_data, 'c')

% legend('MPCC 25','MPC 25','MPC 25 h', 'MPC 50','MPC25 s','MPC50 s')
legend('MPCC 25','MPC 25', 'MPC 50','MPC 25 s','MPC 50 s')
axis([0 80 1e-15 1e0])

xlabel('Time [s]')
ylabel('KKT value')

figure
load('straight_line_through_obstacle/mpcc25_with_hotstart.mat')
plot(linspace(0,length(time_mpc_data),length(time_mpc_data)),time_mpc_data,'r')

load('straight_line_through_obstacle/mpc25_with_hotstart.mat')
hold on
plot(linspace(0,length(time_mpc_data),length(time_mpc_data)),time_mpc_data,'b')
% 
% load('straight_line_through_obstacle/mpc25_no_hotstart.mat')
% plot(linspace(0,length(time_mpc_data),length(time_mpc_data)),time_mpc_data,'g')

load('straight_line_through_obstacle/mpc50_with_hotstart.mat')
plot(linspace(0,length(time_mpc_data),length(time_mpc_data)),time_mpc_data,'k')

load('straight_line_through_obstacle/mpc25_small_weight.mat')
plot(linspace(0,length(time_mpc_data),length(time_mpc_data)),time_mpc_data,'m')

load('straight_line_through_obstacle/mpc50_small_weight.mat')
plot(linspace(0,length(time_mpc_data),length(time_mpc_data)),time_mpc_data,'c')

% legend('MPCC 25','MPC 25','MPC 25 h', 'MPC 50','MPC25 s','MPC50 s')
legend('MPCC 25','MPC 25', 'MPC 50','MPC 25 s','MPC 50 s')

xlabel('Time [s]')
ylabel('Computation time [ms]')
