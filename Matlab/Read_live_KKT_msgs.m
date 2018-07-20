clear all
close all
clc

rosshutdown
rosinit

liveplot = false;

figure

kkt_sub = rossubscriber('/MPC/kkt');
time_sub = rossubscriber('/MPC/solvertime');

disp('Start tracker')

kkt_data(1) = receive(kkt_sub,100);
time_data(1) = time_sub.LatestMessage;

kkt_mpc_data(1) = kkt_data(1).Data;
time_mpc_data(1) = time_data(1).Data; 

if (liveplot)
    figure
    subplot(211)
    semilogy(kkt_mpc_data)
    xlabel('Time [s]')
    ylabel('KKT value')
    subplot(212)
    plot(time_mpc_data)
    xlabel('Time [s]')
    ylabel('Computation time [ms]')
end

for i = 2:1000
    try
        kkt_data(i) = receive(kkt_sub,10);
        time_data(i) = time_sub.LatestMessage;
        
        kkt_mpc_data(i) = kkt_data(i).Data;
        time_mpc_data(i) = time_data(i).Data;
        
        if (liveplot)
            subplot(211)
            semilogy(kkt_mpc_data)
            xlabel('Time [s]')
            ylabel('KKT value')
            subplot(212)
            plot(time_mpc_data)
            xlabel('Time [s]')
            ylabel('Computation time [ms]')
        end
    catch
        break
    end
    disp(i);
end

save('mpc50_small_weight.mat')