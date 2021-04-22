%% Crazyflie Model
%% General Params
m = 0.027;      % mass [Kg]
g = 9.81;       % gravity [m/s2]
d = 39.73e-3;   % arm length [m]
r = 23.1348e-3; % rotor radius [m]
Ixx = 1.657e-5; % moment of inertia [Kg.m-2]
Iyy = 1.665e-5; % moment of inertia [kg.m-2]
Izz = 2.296e-5; % moment of inertia [kg.m-2]
kT = 0.2025;    % adimensional
kD = 0.11;      % adimensional
fm = 0.2685;    % adimensional
ro = 1.225;     % densidad del aire [Kg/m3]

CT = kT*ro*(2*r)^4/3600;  %% [N/rpm2]
CD = kD*ro*(2*r)^5/3600;   %% [N.m/rpm2]

%% Modelo Lineal
we=sqrt((m*g)/(4*CT));        %% [rpm]
PWMe=(we-4070.3)/0.2685;
OMEGA_e=(we-4070.3)/fm;

%Fuerzas y Momentos from velocidades angulares
ML1=2*we*[CT,CT,CT,CT;
    -d*CT/sqrt(2),-d*CT/sqrt(2),d*CT/sqrt(2),d*CT/sqrt(2);
    -d*CT/sqrt(2),d*CT/sqrt(2),d*CT/sqrt(2),-d*CT/sqrt(2);
    -CD,CD,-CD,CD];

ML2=[0,0;
    1,0];

ML3=[1/m;0];

ML4=[0,0;
    1,0];

ML5=[1/Izz;0];

ML6=[0,0,0,0;
    1,0,0,0;
    0,-g,0,0;
    0,0,1,0];

ML7=[1/Ixx;0;0;0];

ML8=[0,0,0,0;
    1,0,0,0;
    0,g,0,0;
    0,0,1,0];

ML9=[1/Iyy;0;0;0];

% Modelo de motor OMEGAi=0.2685PWMi+4070.3
%% Modelo NO LINEAL
% MNL1 = [Fi;Mx;My;Mz]
MNL1=[         CT,           CT,           CT,           CT;
    -d*CT/sqrt(2),-d*CT/sqrt(2), d*CT/sqrt(2), d*CT/sqrt(2);
    -d*CT/sqrt(2), d*CT/sqrt(2), d*CT/sqrt(2),-d*CT/sqrt(2);
              -CD,           CD,          -CD,           CD];