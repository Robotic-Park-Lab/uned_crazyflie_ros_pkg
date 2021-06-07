%%

%% Rate Controller
Dpitch.N = length(rate_controller.dpitch.Data);
[Dpitch.IAE, Dpitch.ITAE] = CF_IAE(rate_controller.dpitch_error.Data, rate_controller.dpitch_error.Time);
Dpitch.Table = table([Dpitch.N;Dpitch.IAE;Dpitch.ITAE],'VariableNames',{'Dpitch Controller'});
Droll.N = length(rate_controller.droll.Data);
[Droll.IAE, Droll.ITAE] = CF_IAE(rate_controller.droll_error.Data, rate_controller.droll_error.Time);
Droll.Table = table([Droll.N;Droll.IAE;Droll.ITAE],'VariableNames',{'Droll Controller'});
Dyaw.N = length(dyaw_controller.dyaw.Data);
[Dyaw.IAE, Dyaw.ITAE] = CF_IAE(dyaw_controller.dyaw_error.Data, dyaw_controller.dyaw_error.Time);
Dyaw.Table = table([Dyaw.N;Dyaw.IAE;Dyaw.ITAE],'VariableNames',{'Dyaw Controller'});

%% Attitude Controller
Pitch.N = length(attitude_controller.dpitch_ref.Data);
[Pitch.IAE, Pitch.ITAE] = CF_IAE(attitude_controller.pitch_error.Data, attitude_controller.pitch_error.Time);
Pitch.Table = table([Pitch.N;Pitch.IAE;Pitch.ITAE],'VariableNames',{'Pitch Controller'});
Roll.N = length(attitude_controller.droll_ref.Data);
[Roll.IAE, Roll.ITAE] = CF_IAE(attitude_controller.roll_error.Data, attitude_controller.roll_error.Time);
Roll.Table = table([Roll.N;Roll.IAE;Roll.ITAE],'VariableNames',{'Roll Controller'});

%% Yaw Controller
Yaw.N = length(yaw_controller.control_signal.Data);
[Yaw.IAE, Yaw.ITAE] = CF_IAE(yaw_controller.yaw_error.Data, yaw_controller.yaw_error.Time);
Yaw.Table = table([Yaw.N;Yaw.IAE;Yaw.ITAE],'VariableNames',{'Yaw Controller'});

%% Altitude Controller
Z.N = length(altitude_controller.omega.Data);
[Z.IAE, Z.ITAE] = CF_IAE(altitude_controller.z_error.Data, altitude_controller.z_error.Time);
Z.Table = table([Z.N;Z.IAE;Z.ITAE],'VariableNames',{'Altitude Controller'},'RowName',{'Samples','IAE','ITAE'});

%% X-Y Controller
X.N = length(x_controller.pitch_ref.Data);
[X.IAE, X.ITAE] = CF_IAE(x_controller.xbe.Data, x_controller.xbe.Time);
X.Table = table([X.N;X.IAE;X.ITAE],'VariableNames',{'X Controller'});
Y.N = length(y_controller.roll_ref.Data);
[Y.IAE, Y.ITAE] = CF_IAE(y_controller.ybe.Data, y_controller.ybe.Time);
Y.Table = table([Y.N;Y.IAE;Y.ITAE],'VariableNames',{'Y Controller'});

aux = [Z.Table X.Table Y.Table Yaw.Table Pitch.Table Roll.Table Dpitch.Table Droll.Table Dyaw.Table];
results.(['controller',num2str(i)]) = aux;
disp(aux)


if(i==comparative && comparative ~= 1 )
   for data = 2:comparative
       [f,c] = size(results.(['controller',num2str(data)]));
       results.(['controller',num2str(comparative+data)]) = array2table(zeros(f,c));
       results.(['controller',num2str(comparative+data)]).Properties.VariableNames = results.(['controller',num2str(1)]).Properties.VariableNames;
       results.(['controller',num2str(comparative+data)]).Properties.RowNames = results.(['controller',num2str(1)]).Properties.RowNames;
       for k = 1:c
          for l = 1:f
              results.(['controller',num2str(comparative+data)]){l,k} = (1-results.(['controller',num2str(data)]){l,k}/results.(['controller',num2str(1)]){l,k})*100;
          end
       end
   end
end