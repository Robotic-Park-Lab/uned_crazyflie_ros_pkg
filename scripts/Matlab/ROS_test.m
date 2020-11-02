%% Script para la lectura de datos desde una red de ROS
clear all
clc
close all
%% Conexión a red existente de ROS
% % Dirección de la red creada en la máquina virtual diseñada:
% ROS_MASTER_URI = 'http://rosdoctorado-VirualBox:11311';
% % Si se desea que sea el usuario quien lo defina, descomentar L9-10
% % MASTER_URI = 'What is the ROS__MASTER_URI? ';
% % ROS_MASTER_URI = input(MASTER_URI,'s');
% rosinit(ROS_MASTER_URI,'NodeName','/test_node');

% Conexión por IP
rosinit('172.20.10.3')
%% Crear nuevos nodos
node = ros.Node('/test_node_control');
lector = ros.Node('/test_node_lector');
graph_node = ros.Node('/test_node_graph');

%% Publicación y Subscripción de topics en la red
crazyflie_pub = ros.Publisher(node,'/Crazyflie_test/name','std_msgs/String');
crazyflie_pos_pub = ros.Publisher(node,'/Crazyflie_test/position','geometry_msgs/Point');
crazyflie_sub = ros.Subscriber(lector,'/Crazyflie_test/name','std_msgs/String');
crazyflie_pos_sub = ros.Subscriber(graph_node,'/Crazyflie_test/position','geometry_msgs/Point');

pause(1)

msg_position = rosmessage('geometry_msgs/Point');
msg_position.X = 0.1;
msg_position.Y = 0.2;
msg_position.Z = -0.1;
send(crazyflie_pos_pub,msg_position);

msg_string = rosmessage('std_msgs/String');
msg_string.Data = 'Anibal';
send(crazyflie_pub,msg_string);

crazyflie_sub.LatestMessage
crazyflie_pos_sub.LatestMessage
%% Ejecución del código

pause

%% El programa debe cerrarse eliminando los nodos y topics creados y eliminando el nodo global con el que se inicia el programa
clear('crazyflie_pub','crazyflie_sub','crazyflie_pos_pub','crazyflie_pos_sub')
clear('node','lector','graph_node')
rosshutdown

