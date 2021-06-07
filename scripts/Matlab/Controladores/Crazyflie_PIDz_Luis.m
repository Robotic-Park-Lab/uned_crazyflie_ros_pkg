%% Crazyflie PID(Z) Controllers% X_T -> Period of signal% X_q = [Q0; Q1; Q2] -> PID(z) controllerCrazyflie_PIDs_Luisif(contains(controladores(controllers(i),1),'PIDz'))    disp('Discrete PID controller')    str = strcat(id,' PID(z)');    controladores(controllers(i),2) = str;end%% Frequenciesfs_inner= 500;  % 500 Hzfs_outer= 250;  % 250 Hzfs_ext  = 100;  % 100 Hz%% CONTROLER MIXER: MatrixMCO =[1,-1/2,-1/2,-1;      1, 1/2,-1/2, 1;      1, 1/2, 1/2,-1;      1,-1/2, 1/2, 1];  %% RATE CONTROLLER% PID DphiDphi_T = 1/fs_inner;Dphi_q = PID_c2d(Dphi_s,Dphi_T);% PID DthetaDtheta_T = 1/fs_inner;Dtheta_q = PID_c2d(Dtheta_s,Dtheta_T);% PID DpsiDpsi_T = 1/fs_inner;Dpsi_q = PID_c2d(Dpsi_s,Dpsi_T);%% ATTITUDE CONTROLLERphi_theta_limit = 180;% PID phiPhi_T = 1/fs_inner;Phi_q = PID_c2d(Phi_s,Phi_T);% PID thetaTheta_T = 1/fs_inner;Theta_q = PID_c2d(Theta_s,Theta_T);%% Off Board Controller% YAW CONTROLLERYaw_T = 1/fs_ext;Yaw_q = PID_c2d(Yaw_s,Yaw_T);% ALTITUDE CONTROLLERZ_T = 1/fs_ext;Z_q = PID_c2d(Z_s,Z_T);W_T = 1/fs_ext;W_q = PID_c2d(W_s,W_T);% X-Y POSITION CONTROLLER% Limit control signalxyv_lim = 30; % [rad/s]% X -> phiX_T = 1/fs_ext;X_q = PID_c2d(X_s,X_T);U_T = 1/fs_ext;U_q = PID_c2d(U_s,U_T);% Y -> thetaY_T = 1/fs_ext;Y_q = PID_c2d(Y_s,Y_T);V_T = 1/fs_ext;V_q = PID_c2d(V_s,V_T);