%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CRAZYFLIE 2.1 Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables;
% clc;
close all;

disp('***********************')
disp('*    Crazyflie 2.1    *')
disp('***********************')
disp('')
% PIDc  -> PID continuo
% PIDd  -> PID discreto
% PIDeb -> PID Basado en Eventos
controller = "PIDeb";
%% Model
disp('Modelling ...')
tic
Crazyflie_Model
tm = toc;
fprintf('\tModelling time: %f\n', tm)
%% Controller
% PID Continuo
if(controller == "PIDc")
    disp('Continuous PID controller')
    disp('Loading parameters ... ')
    tic
    Crazyflie_PIDs
    tm = toc;
    fprintf('\tLoading parameters time: %f\n', tm)
    disp('Simulating ... ')
    tic
    sim('Crazyflie_NoLinealModel_S')
    tm = toc;
    fprintf('\tSimulation time: %f\n', tm)
    fprintf('\tSamples: \n\t\tAltitude Controller: %d\n\t\tRate Controller: %d\n\t\tAttitude Controller: %d\n\t\tX-Y Position Controller: %d\n\t\tYaw Controller: %d\n', length(altitude_controller.omega.Data), length(rate_controller.Droll.Data),length(attitude_controller.dpitch_ref.Data),length(x_controller.pitch_ref.Data),length(yaw_controller.control_signal.Data))
end
% PID Discreto
if(controller == "PIDd")
    disp('Discrete PID controller')
    disp('Loading parameters ... ')
    tic
    Crazyflie_PIDz
    tm = toc;
    fprintf('\tLoading parameters time: %f\n', tm)
    disp('Simulating ... ')
    tic
    sim('Crazyflie_NoLinealModel_Z')
    tm = toc;
    fprintf('\tSimulation time: %f\n', tm)
    fprintf('\tSamples: \n\t\tAltitude Controller: %d\n\t\tRate Controller: %d\n\t\tAttitude Controller: %d\n\t\tX-Y Position Controller: %d\n\t\tYaw Controller: %d\n', length(altitude_controller.omega.Data), length(rate_controller.Droll.Data),length(attitude_controller.dpitch_ref.Data),length(x_controller.pitch_ref.Data),length(yaw_controller.control_signal.Data))
end
% PID Basado en Eventos
if(controller == "PIDeb")
    disp('Event based PID controller')
    disp('Loading parameters ... ')
    tic
    Crazyflie_PIDeb
    tm = toc;
    fprintf('\tLoading parameters time: %f\n', tm)
    disp('Simulating ... ')
    tic
    sim('Crazyflie_NoLinealModel_EB')
    tm = toc;
    figure
    aux = diff(altitude_controller.omega.Time);
    plot(aux)
    plot(altitude_controller.omega.Time(1:end-1),aux,'.')
    fprintf('\tSimulation time: %f\n', tm)
    fprintf('\tSamples: \n\t\tAltitude Controller: %d\n\t\tRate Controller: %d\n\t\tAttitude Controller: %d\n\t\tX-Y Position Controller: %d\n\t\tYaw Controller: %d\n', length(altitude_controller.omega.Data), length(rate_controller.Droll.Data),length(attitude_controller.dpitch_ref.Data),length(x_controller.pitch_ref.Data),length(yaw_controller.control_signal.Data))
end
%% Graphs
disp('Graphs ... ')
tic
Crazyflie_Graphs
tm = toc;
fprintf('\tGraphs time: %f\n', tm)