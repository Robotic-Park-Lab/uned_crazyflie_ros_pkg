%% Script con un controlador de muestra para el repositorio CrazyS
clear all
clc
close all
rosshutdown

%%
setenv('ROS_HOSTNAME','192.168.1.51')
setenv('ROS_MASTER_URI','http://192.168.1.66:11311')

%The vechile parameters
maxRotorsVelocity = 2618; %[rad/s]
Omega_e = 6874; %[rad/s]

%A new connection is estabilished with the ROS master
rosinit

%Topics will be used during the simulation
sub = rossubscriber('/crazyflie2/odometry_sensor1/odometry'); % contains the drone state
%sub = rossubscriber('/crazyflie2/odometry_sensor1/odometry','nav_msgs/Odometry'); % contains the drone state
sub2 = rossubscriber('/crazyflie2/command/motor_speed'); 
%sub2 = rossubscriber('/crazyflie2/command/motor_speed','mav_msgs/Actuators'); 
[pubCmd, msgCmd] = rospublisher('/crazyflie2/command/motor_speed','mav_msgs/Actuators');

%ROS service employed during the aircraft simulation. In particular, the
%service pause and unpause physic allow to pause and start the simulation,
%respectively
pauseROS = rossvcclient('/gazebo/pause_physics');
unpauseROS = rossvcclient('/gazebo/unpause_physics');
time = rossvcclient('/gazebo/get_world_properties');
physicProperties = rossvcclient('/gazebo/get_physics_properties');

%Information about the simulation
physicPropertiesObject = call(physicProperties);
TimeStepGazebo = physicPropertiesObject.TimeStep;
PauseGazebo = physicPropertiesObject.Pause;
MaxUpdateRateGazebo = physicPropertiesObject.MaxUpdateRate;

%The integrators initial values
intPosXX = 0;
intPosYY = 0;
intAttTheta = 0;
intAttPhi = 0;
intRateRate = 0;
intZZ = 0;

%Linear regression coefficients
motorsIntercepts = 426.24; %[rad/s]
motorAngularCoefficient = 0.2685;

%Position references
x_ref = 0; %[m]
y_ref = 0; %[m]
z_ref = 1; %[m]
psi_ref = 0; %[rad]

%The simulation step
stop = 0;
T = 0.01; %Sampling time

%The controller parameter values
Kp_YawPositionController = 0.0914; 
Kp_ThetaC = 3.594;
Kp_PhiC = -3.594;
Kp_pc = 0.0611;
Kp_qc = 0.0611;
Kp_altitude = 70;
Kp_deltaPhi = 1000;
Kp_deltaTheta = 1000;
Kp_deltaPsi = 1000;

Ki_ThetaC = 5.72958;
Ki_PhiC = -5.72958;
Ki_pc = 0.0349;
Ki_qc = 0.0349;
Ki_altitude = 3.15;
Ki_deltaPsi = 95.6839;

Kd_altitude = 373;

%The bound limits
yawRateBound = 3.4907; %[rad/s]
thetaCBound = pi/6; %[rad]
phiCBound = pi/6; %[rad]
deltaOmegaPosBound = 1289; %[PWM]
deltaOmegaNegBound = -1718; %[PWM]

%The step size of the simulation. In other words, the number of steps the
%simulation will run
simulationSteps = 500;

%To take track of the simulation history
x_vector = zeros(simulationSteps, 1);
y_vector = zeros(simulationSteps, 1);
z_vector = ones(simulationSteps, 1);

phi_vector = zeros(simulationSteps, 1);
theta_vector = zeros(simulationSteps, 1);
psi_vector = zeros(simulationSteps, 1);

q0_vector = zeros(simulationSteps, 1);
q1_vector = zeros(simulationSteps, 1);
q2_vector = zeros(simulationSteps, 1); 
q3_vector = zeros(simulationSteps, 1);

u_vector = zeros(simulationSteps, 1);
v_vector = zeros(simulationSteps, 1);
w_vector = zeros(simulationSteps, 1);

p_vector = zeros(simulationSteps, 1);
q_vector = zeros(simulationSteps, 1);
r_vector = zeros(simulationSteps, 1);

motor_velocities_GAZEBO = zeros(simulationSteps, 4);
motor_velocities_SIMULINK = zeros(simulationSteps, 4);

%To take track of the intregrators values
intPosXX_vector = zeros(simulationSteps, 1);
intPosYY_vector = zeros(simulationSteps, 1);
intAttTheta_vector = zeros(simulationSteps, 1);
intAttPhi_vector = zeros(simulationSteps, 1);
intRateRate_vector = zeros(simulationSteps, 1);
intZZ_vector = zeros(simulationSteps, 1);

%To take track of the errors and integrators output
altitude_error_vector = zeros(simulationSteps, 1);
proportionalContribute = zeros(simulationSteps, 1);
integrativeContribute = zeros(simulationSteps, 1);
derivativeContribute = zeros(simulationSteps, 1);

%The altitude controller output
omega_vector = zeros(simulationSteps, 1);

%The tracjetory error in the body-frame
trajectoryErrorX_vector = zeros(simulationSteps, 1);
trajectoryErrorY_vector = zeros(simulationSteps, 1);

xe_vector = zeros(simulationSteps, 1);
ye_vector = zeros(simulationSteps, 1);

%The propellers angular velocity values. It is used as reference by the
%motors
angVel_SIMULINK_vector = zeros(simulationSteps, 4);

%Cell initialization
messageNumber = 1;
GazeboMessage = cell(simulationSteps, messageNumber);

%The time before the simulation starts
serviceTime = call(time);
startTime= serviceTime.SimTime;

%Other variables of interest
theta_c_vector = zeros(simulationSteps, 1);
phi_c_vector = zeros(simulationSteps, 1);

errorTheta_c_vector = zeros(simulationSteps, 1);
errorPhi_c_vector = zeros(simulationSteps, 1);

pc_vector = zeros(simulationSteps, 1);
qc_vector = zeros(simulationSteps, 1);
rc_vector = zeros(simulationSteps, 1);

errorPc_vector = zeros(simulationSteps, 1);
errorQc_vector = zeros(simulationSteps, 1);
errorRc_vector = zeros(simulationSteps, 1);

delta_theta_vector = zeros(simulationSteps, 1);
delta_phi_vector = zeros(simulationSteps, 1);
delta_psi_vector = zeros(simulationSteps, 1);

eul_vector = zeros(simulationSteps, 3);

%The PWM vector values during the simulation
PWM_1_vector = zeros(simulationSteps, 1);
PWM_2_vector = zeros(simulationSteps, 1);
PWM_3_vector = zeros(simulationSteps, 1);
PWM_4_vector = zeros(simulationSteps, 1);

%The omega values during the simulation
Omega_1_vector = zeros(simulationSteps, 1);
Omega_2_vector = zeros(simulationSteps, 1);
Omega_3_vector = zeros(simulationSteps, 1);
Omega_4_vector = zeros(simulationSteps, 1);


for i=1:simulationSteps
    
    %The message from Gazebo
    msgS  = receive(sub);
    %msgSS = receive(subMotor);
    
    %Gazebo has stopped
    call(pauseROS);
    
    %The simulation time later the simulation stop
    serviceTime = call(time);
    simulationTimeMeasurement = serviceTime.SimTime;
    if i == 1
        timeOffset = simulationTimeMeasurement;
    end
    
    %The start and stop time fixing
    start = simulationTimeMeasurement - timeOffset;
    %start = stop;
    stop = start + T;
    
    %Number of steps printed on the screen
    i
	    
    %The vector of messages
    GazeboMessage{i,1} = msgS;
    
    %The propellers angular velocities storing
    %motor_velocities_GAZEBO(i,:) = msgSS.AngularVelocities;
    
    %Position
    x_GAZEBO = msgS.Pose.Pose.Position.X;
    y_GAZEBO = msgS.Pose.Pose.Position.Y;
    z_GAZEBO = msgS.Pose.Pose.Position.Z;
    
    x_vector(i,1) = x_GAZEBO;
    y_vector(i,1) = y_GAZEBO;
    z_vector(i,1) = z_GAZEBO;
    
    %Linear velocity
    u_GAZEBO = msgS.Twist.Twist.Linear.X;
    v_GAZEBO = msgS.Twist.Twist.Linear.Y;
    w_GAZEBO = msgS.Twist.Twist.Linear.Z;
    
    u_vector(i,1) = u_GAZEBO;
    v_vector(i,1) = v_GAZEBO;
    w_vector(i,1) = w_GAZEBO;
    
    %Quaterion (they define the aircraft attitude)
    q0_GAZEBO = msgS.Pose.Pose.Orientation.W;
    q1_GAZEBO = msgS.Pose.Pose.Orientation.X;
    q2_GAZEBO = msgS.Pose.Pose.Orientation.Y;
    q3_GAZEBO = msgS.Pose.Pose.Orientation.Z;
    
    q0_vector(i,1) = q0_GAZEBO;
    q1_vector(i,1) = q1_GAZEBO;
    q2_vector(i,1) = q2_GAZEBO;
    q3_vector(i,1) = q3_GAZEBO;
    
    %The angular veocities of the drone in ABC frame
    p_GAZEBO = msgS.Twist.Twist.Angular.X;
    q_GAZEBO = msgS.Twist.Twist.Angular.Y;
    r_GAZEBO = msgS.Twist.Twist.Angular.Z;
    
    p_vector(i,1) = p_GAZEBO;
    q_vector(i,1) = q_GAZEBO;
    r_vector(i,1) = r_GAZEBO;
    
    %The quaternion is converted in Euler angles
    quat = [q0_GAZEBO q1_GAZEBO q2_GAZEBO q3_GAZEBO];
    EulerAngles = quat2eul(quat, 'ZYX'); % YAW ROLL PITCH
    eul_vector(i,:) = EulerAngles; 
    
    yaw_MATLAB = EulerAngles(1,1);
    pitch_MATLAB = EulerAngles(1,2);
    roll_MATLAB = EulerAngles(1,3);
    
    %The Simulink scheme starts
    mdl = 'ROS';
    simOut = sim(mdl);
    
    %The trajectory errors are stored in vectr to b analyzed later
    trajectoryErrorX_vector(i,1) = trajectoryErrorX.signals.values(end);
    trajectoryErrorY_vector(i,1) = trajectoryErrorY.signals.values(end);
    
    xe_vector(i,1) = xe_SIMULINK.signals.values(end);
    ye_vector(i,1) = ye_SIMULINK.signals.values(end);
    
    %The vector contains the propellers angular velocities
    motor_velocities_SIMULINK(i,:) = [Omega_1_SIMULINK.signals.values(end),...
        Omega_2_SIMULINK.signals.values(end), Omega_3_SIMULINK.signals.values(end), Omega_4_SIMULINK.signals.values(end)];
    
    %FromWorkSpace Simulink
    theta_vector(i,1) = pitch_MATLAB;
    phi_vector(i,1) = roll_MATLAB;
    psi_vector(i,1) = yaw_MATLAB; 
    
    %Other useful data coming from the simulation
    phi_c_vector(i,1) = phi_c_SIMULINK.signals.values(end);
    theta_c_vector(i,1) = theta_c_SIMULINK.signals.values(end);
    
    errorTheta_c_vector = errorTheta_c_SIMULINK.signals.values(end);
    errorPhi_c_vector = errorPhi_c_SIMULINK.signals.values(end);
    
    pc_vector(i,1) = pc_SIMULINK.signals.values(end);
    qc_vector(i,1) = qc_SIMULINK.signals.values(end);
    rc_vector(i,1) = rc_SIMULINK.signals.values(end);
    
    errorPc_vector(i,1) = errorPc_SIMULINK.signals.values(end);
    errorQc_vector(i,1) = errorQc_SIMULINK.signals.values(end);
    errorRc_vector(i,1) = errorRc_SIMULINK.signals.values(end);
    
    delta_theta_vector(i,1) = delta_theta_SIMULINK.signals.values(end);
    delta_psi_vector(i,1) = delta_psi_SIMULINK.signals.values(end);
    delta_phi_vector(i,1) = delta_phi_SIMULINK.signals.values(end);
    
    %Data of the altitude controller
    altitude_error_vector(i,1) = altitudeError.signals.values(end);
    proportionalContribute(i,1) = intZP.signals.values(end);
    integrativeContribute(i,1) = intZI.signals.values(end);
    derivativeContribute(i,1) = intZD.signals.values(end);
    
    omega_vector(i,1) = Omega.signals.values(end);
    
    %Also the PWM values are stored into files
    PWM_1_vector(i,1) = PWM_1_SIMULINK.signals.values(end);
    PWM_2_vector(i,1) = PWM_2_SIMULINK.signals.values(end);
    PWM_3_vector(i,1) = PWM_3_SIMULINK.signals.values(end);
    PWM_4_vector(i,1) = PWM_4_SIMULINK.signals.values(end);
    
    %The integral values are updated
    intPosXX = intPosXX_SIMULINK.signals.values(end);
    intPosYY = intPosYY_SIMULINK.signals.values(end);
    intAttTheta = intAttTheta_SIMULINK.signals.values(end);
    intAttPhi = intAttPhi_SIMULINK.signals.values(end);
    intRateRate = intRateRate_SIMULINK.signals.values(end);
    intZZ = intZZ_SIMULINK.signals.values(end);
    
    %The ingrators history
    intPosXX_vector(i,1) = intPosXX;
    intPosYY_vector(i,1) = intPosYY;
    intAttTheta_vector(i,1) = intAttTheta;
    intAttPhi_vector(i,1) = intAttPhi;
    intRateRate_vector(i,1) = intRateRate;
    intZZ_vector(i,1) = intZZ;
    
    %Also the omega values (the propelles angular velocities) are stored
    %into files. The history could improve the controller performances
    Omega_1_vector(i,1) = Omega_1_SIMULINK.signals.values(end);
    Omega_2_vector(i,1) = Omega_2_SIMULINK.signals.values(end);
    Omega_3_vector(i,1) = Omega_3_SIMULINK.signals.values(end);
    Omega_4_vector(i,1) = Omega_4_SIMULINK.signals.values(end);
    
    %The new omega values are sent to Gazebo before remove it to pause
    msgCmd.AngularVelocities = [Omega_1_SIMULINK.signals.values(end),...
        Omega_2_SIMULINK.signals.values(end), Omega_3_SIMULINK.signals.values(end), Omega_4_SIMULINK.signals.values(end)];
    
    send(pubCmd,msgCmd)
    showdetails(msgCmd)
    
    %Until the number of steps is not equal to 500
    if(i == 500)
        serviceTime = call(time);
        stopSimuationTime = serviceTime.SimTime;
    end
    
    %Gazebo exits from pause
    call(unpauseROS);
    
end