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

% MODIFIED VERSION by Carlos Luis, 2016
quad.nrotors = 4;                %   4 rotors
quad.g = 9.81;                   %   g       Gravity                             1x1
quad.rho = 1.225;                %   rho     Density of air                      1x1
quad.muv = 1.5e-5;               %   muv     Viscosity of air                    1x1

% Airframe
quad.M = 0.03327; %0.029; %0.03327;                      %   M       Mass                                1x1
Ixx =  1.395e-05;  %2.3951e-5; %1.39e-05; %2.51943e-5; % % %
Iyy = 1.395e-05; %2.3951e-5; %1.436e-05; % 2.54379e-5; % %         %
Izz =  2.173e-05;  %3.2347e-5; %2.173e-05 ; %4.51407e-5; % %         %;%0.160;
quad.J = diag([Ixx Iyy Izz]);    %   I       Flyer rotational inertia matrix     3x3

quad.h = -0.00336747;                 %   h       Height of rotors above CoG          1x1
quad.d = 39.73e-3;                  %   d       Length of flyer arms                1x1

%Rotor
quad.nb = 2;                      %   b       Number of blades per rotor          1x1
quad.r =23.1348e-3;                  %   r       Rotor radius                        1x1
%quad.r = quad.r*sin(pi/4); %Just to flight in X mode

quad.c = 9.59e-3;                  %   c       Blade chord                         1x1

quad.Ct = 0.15; %0.0187336 %1.92558e-03;                    %2.11163e-05 for rad/s                            %            %   Ct      Non-dim. thrust coefficient         1x1
quad.Cq = 0.11; % quad.Ct*sqrt(quad.Ct/2);         %   Cq      Non-dim. torque coefficient         1x1
quad.Mb = 0.00025                     %Rotor Blade Mass
quad.Jr = 0.5*quad.Mb*quad.r^2;


% derived constants
quad.A = pi*quad.r^2;                 %   A       Rotor disc area                     1x1

quad.b = 1.35*quad.Ct*quad.rho*(2*pi*9.5493)^(-2)*(2*quad.r)^4; %1.55 for quad without UWB T = b w^2 1.16
quad.k = quad.Cq*quad.rho*(2*pi*9.5493)^(-2)*(2*quad.r)^5; % Q = k w^2

quad.verbose = false;

%% Definition of state model

x_e = [0 0 0 0 0 0 0 0 0 0 0 0];
we = sqrt(quad.M*quad.g/(4*quad.b));
pwm = (we-4070.3)/0.2685;
% u = [we we we we];
u=[pwm pwm pwm pwm];

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