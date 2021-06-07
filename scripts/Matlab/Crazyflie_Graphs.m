%% Crazyflie Graphs results
%% GRAPHS
figure('Name','Position & Yaw')
subplot(4,1,1)
plot(references.x_ref)
hold on
plot(dron_state.x)
xlabel('Time [s]')
ylabel('X [m]')
title('X position')
legend('Ref', 'Real', 'Location', 'best')
grid minor
subplot(4,1,2)
plot(references.y_ref)
hold on
plot(dron_state.y)
xlabel('Time [s]')
ylabel('Y [m]')
title('Y position')
legend('Ref', 'Real', 'Location', 'best')
grid minor
subplot(4,1,3)
plot(references.z_ref)
hold on
plot(dron_state.z)
xlabel('Time [s]')
ylabel('Z [m]')
title('Z position')
legend('Ref', 'Real', 'Location', 'best')
grid minor
subplot(4,1,4)
plot(references.yaw_ref)
hold on
plot(dron_state.yaw*180/pi)
xlabel('Time [s]')
ylabel('Yaw [º]')
title('Yaw orientation')
legend('Ref', 'Real', 'Location', 'best')
grid minor

% % Altitude Controller
% figure('Name', 'Altitude Controller')
% subplot(2,1,1)
% plot(references.z_ref)
% hold on
% plot(dron_state.z)
% xlabel('Time [s]')
% ylabel('Z [m]')
% title('Z position')
% legend('Ref', 'Real', 'Location', 'best')
% grid minor
% subplot(2,1,2)
% plot(altitude_controller.omega)
% xlabel('Time [s]')
% ylabel('\Omega [rad/s]')
% title('Control Signal')
% grid minor
% 
% % Yaw Controller
% figure('Name', 'Yaw Controller')
% subplot(3,1,1)
% plot(references.yaw_ref)
% hold on
% plot(dron_state.yaw*180/pi)
% xlabel('Time [s]')
% ylabel('Yaw [º]')
% title('Yaw position')
% legend('Ref','Real', 'Location', 'best')
% grid minor
% subplot(3,1,2)
% plot(yaw_controller.control_signal)
% hold on 
% plot(dyaw_controller.dyaw)
% xlabel('Time [s]')
% ylabel('r [º/s]')
% title('Yaw Rate')
% legend('Ref','Real', 'Location', 'best')
% grid minor
% subplot(3,1,3)
% plot(dyaw_controller.control_signal)
% xlabel('Time [s]')
% ylabel('\Delta\Psi [rad]')
% title('Control Signal')
% grid minor
% 
% % X-Y Loop
% % X
% figure('Name', 'X Controller')
% subplot(3,1,1)
% plot(references.x_ref)
% hold on
% plot(dron_state.x)
% hold on
% plot(x_controller.xbe)
% xlabel('Time [s]')
% ylabel('X [m]')
% title('X position')
% legend('Ref','Real','Error','Location', 'best')
% grid minor
% subplot(3,1,2)
% plot(x_controller.uc)
% hold on 
% plot(x_controller.u)
% xlabel('Time [s]')
% ylabel('u [m/s]')
% title('X Rate')
% legend('Ref','Real', 'Location', 'best')
% grid minor
% subplot(3,1,3)
% plot(x_controller.pitch_ref)
% xlabel('Time [s]')
% ylabel('\theta [rad]')
% title('Control Signal')
% grid minor
% % Y
% figure('Name', 'Y Controller')
% subplot(3,1,1)
% plot(references.y_ref)
% hold on
% plot(dron_state.y)
% hold on
% plot(y_controller.ybe)
% xlabel('Time [s]')
% ylabel('Y [m]')
% title('Y position')
% legend('Ref','Real','Error','Location', 'best')
% grid minor
% subplot(3,1,2)
% plot(y_controller.vc)
% hold on 
% plot(y_controller.v)
% xlabel('Time [s]')
% ylabel('v [m/s]')
% title('Y Rate')
% legend('Ref','Real', 'Location', 'best')
% grid minor
% subplot(3,1,3)
% plot(y_controller.roll_ref)
% xlabel('Time [s]')
% ylabel('\phi [º]')
% title('Control Signal')
% grid minor
% 
% % Pitch Loop
% figure('Name', 'Pitch Controller')
% subplot(3,1,1)
% plot(attitude_controller.pitch_ref)
% hold on
% plot(attitude_controller.pitch)
% xlabel('Time [s]')
% ylabel('\theta [º]')
% title('Pitch')
% legend('Ref','Real','Location', 'best')
% grid minor
% subplot(3,1,2)
% plot(rate_controller.dpitch_ref)
% hold on 
% plot(rate_controller.dpitch)
% xlabel('Time [s]')
% ylabel('q [º/s]')
% title('d(\theta)/dt')
% legend('Ref','Real', 'Location', 'best')
% grid minor
% subplot(3,1,3)
% plot(rate_controller.Dpitch)
% xlabel('Time [s]')
% ylabel('\Delta\theta [rad/s]')
% title('Control signal')
% grid minor
% 
% % Roll Loop
% figure('Name', 'Roll Controller')
% subplot(3,1,1)
% plot(attitude_controller.roll_ref)
% hold on
% plot(attitude_controller.roll)
% xlabel('Time [s]')
% ylabel('\phi [º]')
% title('Roll')
% legend('Ref','Real','Location', 'best')
% grid minor
% subplot(3,1,2)
% plot(rate_controller.dpitch_ref)
% hold on 
% plot(rate_controller.dpitch)
% xlabel('Time [s]')
% ylabel('p [º/s]')
% title('d(\phi)/dt')
% legend('Ref','Real', 'Location', 'best')
% grid minor
% subplot(3,1,3)
% plot(rate_controller.Droll)
% xlabel('Time [s]')
% ylabel('\Delta\phi [rad/s]')
% title('Control signal [\Delta\phi]')
% grid minor

%% 3D Results. Crazyflie flight
figure('Name', 'Crazyflie flight')
%subplot(2,1,1)
plot3(references.x_ref.Data,references.y_ref.Data, references.z_ref.Data,'linewidth',1.0,'marker','.')
hold on
plot3(dron_state.x.Data,dron_state.y.Data, dron_state.z.Data,'linewidth',1.0,'marker','.')
grid minor
axis equal
title('Crazyflie flight')
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
legend('Ref','dron', 'Location', 'best')

figure
%subplot(2,1,2)
plot(actuators.w1)
hold on
plot(actuators.w2)
hold on
plot(actuators.w3)
hold on
plot(actuators.w4)
grid minor
xlabel('Time [s]')
ylabel('\omega [rad/s]')
legend('M1','M2','M3','M4', 'Location', 'best')
title('Actuators')
