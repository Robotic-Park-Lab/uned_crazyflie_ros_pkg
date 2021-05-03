%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CRAZYFLIE 2.1 Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables;
clc;
close all;

%% Model
Crazyflie_Model
%% PID Continuo
% Crazyflie_PIDs
% sim('Crazyflie_NoLinealModel_S')
%% PID Discreto
Crazyflie_PIDz
sim('Crazyflie_NoLinealModel_Z')

% Graphs
Crazyflie_Graphs