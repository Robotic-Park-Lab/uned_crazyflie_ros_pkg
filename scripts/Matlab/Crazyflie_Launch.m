%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% CRAZYFLIE 2.1 Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
clc;
close all;

%%
% Model
Crazyflie_Model
% Controllers
Crazyflie_PIDcontrollers

% Simulation
sim('Crazyflie_NoLinealModel')

% Graphs
Crazyflie_Graphs