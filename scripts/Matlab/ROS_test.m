%% Script para comprobar la correcta comunicación con la red de ROS
clear all
clc
close all
rosshutdown
%% Configuración de parámetros de la red
setenv('ROS_HOSTNAME','192.168.1.51')
setenv('ROS_MASTER_URI','http://192.168.1.66:11311')
rosinit
node = ros.Node('/matlab_node');

%% Ejemplo de un publicador del tipo mav_msgs/Actuators
% Si se emplea con el launch crazyflie_without_controller.launch, debe
% hacer que se eleve el dron.
pub = ros.Publisher(node,'/crazyflie2/command/motor_speed','mav_msgs/Actuators');
msg = rosmessage('mav_msgs/Actuators');
msg.AngularVelocities = [2290.4; 2290.4; 2290.4; 2290.4];
send(pub,msg);
for i=1:300
    msg.AngularVelocities = [2290.4+i; 2290.4+i; 2290.4+i; 2290.4+i];
    msg.Header.Stamp = rostime('now','system');
    send(pub,msg);
    showdetails(msg)
    pause(0.2)
end

%% Ejemplo de un subscriptor del tipo mav_msgs/Actuators
% Si se emplea con el launch crazyflie_hoovering_example.launch, deben
% observarse las señales de control que recibe el dron.

% crazyflie_sub = ros.Subscriber(node,'/crazyflie2/command/motor_speed','mav_msgs/Actuators');
% crazyflie_sub.LatestMessage
% for i=1:100
%     crazyflie_sub.LatestMessage.AngularVelocities
%     pause(0.1)
% end

%% Ejemplo de un publicador/subscriptor del tipo mav_msgs/Actuators

% pub = ros.Publisher(node,'/motor_speed','mav_msgs/Actuators');
% msg = rosmessage('mav_msgs/Actuators');
% msg.AngularVelocities = [2290.4; 2290.4; 2290.4; 2290.4];
% send(pub,msg);
% crazyflie_sub = ros.Subscriber(node,'/motor_speed','mav_msgs/Actuators');
% crazyflie_sub.LatestMessage
% pause(0.2)
% 
% for i=1:300
%     msg.AngularVelocities = [2290.4+i; 2290.4+i; 2290.4+i; 2290.4+i];
%     msg.Header.Stamp = rostime('now','system');
%     send(pub,msg);
%     showdetails(msg)
%     pause(0.1)
%     crazyflie_sub.LatestMessage.AngularVelocities
%     pause(0.1)
% end

clear('node')
rosshutdown

