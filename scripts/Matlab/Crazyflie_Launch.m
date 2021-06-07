%% CRAZYFLIE 2.1 Set up
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables;
close all;

disp('***********************')
disp('*    Crazyflie 2.1    *')
disp('***********************')
disp('')
%% Model
disp('Modelling ...')
Crazyflie_Model
%% Controller
disp(' ')
comparative=input('Select the numbers of controllers to test: ');
controllers = zeros(comparative,1);
disp(' ')
cd('Controladores')
archivos = dir(fullfile(pwd,'*Crazyflie_PID*'));
controladores = strings(length(archivos),1);
disp('Controllers:')
for k=1:length(archivos)
    controladores(k,1) = cellstr(archivos(k).name);
    fprintf('\t%i. %s\n',k,archivos(k).name);
end
disp(' ')
for i = 1:comparative
    if(comparative>1 && i==1)
        str = ['Select controller ' num2str(i) ' (Reference)[Number]: '];
    else
        str = ['Select controller ' num2str(i) ' [Number]: '];
    end
    controllers(i)=input(str);
end
disp(' ')
cd ..
for i = 1:comparative
    cd('Controladores')
    run(controladores(controllers(i),1))
    cd ..
    %% Simulation
    disp('Simulating ... ')
    % PID Continuo
    if(contains(controladores(controllers(i),1),'PIDs'))
        modelo = 'Crazyflie_NoLinealModel_S';
    end
    % PID Discreto
    if(contains(controladores(controllers(i),1),'PIDz'))
        modelo = 'Crazyflie_NoLinealModel_Z';
    end
    % PID Basado en Eventos
    if(contains(controladores(controllers(i),1),'PIDeb'))
        modelo = 'Crazyflie_NoLinealModel_EB';
    end
    % Matlab version
    vmatlab = version('-release');
    modelo = strcat(modelo,'_',vmatlab);
    cd('Modelos')
    sim(modelo)
    cd ..
    %% Stats, Results & Graphs
    Crazyflie_Stats
    Crazyflie_Graphs
end

for i = comparative+2:comparative*2
    title = strcat('<strong>',controladores(controllers(1),2), " vs ", controladores(controllers(i-comparative),2),'</strong>');
    disp(title)
    disp(results.(['controller',num2str(i)]))
end
